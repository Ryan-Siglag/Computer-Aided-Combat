/**
 * pwm_passthrough.cpp
 *
 * Reads 3 RC PWM input signals (Drive, Turn, Weapon Speed) and outputs
 * 3 PWM signals where:
 *   - Drive  → passed through 1:1, no scaling
 *   - Turn   → passed through 1:1, no scaling
 *   - Weapon → gyro-compensated: speed is reduced as turn rate increases to cap
 *              the destabilising gyroscopic torque  tau = I * omega * Omega.
 *
 * Two scaling modes are available via WEAPON_SCALE_MODE:
 *
 *   LINEAR (mode 0) — fast, simple, easy to tune:
 *       omega = omega0 * (1 - k * |Omega|)
 *       where k = GYRO_K  (recommended starting value: 0.4)
 *       |Omega| = 0  ->  full weapon speed
 *       |Omega| = 1  ->  weapon speed * (1 - k)
 *
 *   HYPERBOLIC (mode 1) — physics-accurate, holds tau = I*omega*Omega <= tau_max:
 *       omega = min(omega0,  GYRO_TAU_LIMIT / |Omega|)
 *       GYRO_TAU_LIMIT = tau_max / I  (tune as a normalised scalar in (0, 1])
 *       Small |Omega|  ->  full weapon speed (clamped to omega0)
 *       Large |Omega|  ->  weapon speed drops sharply on a 1/|Omega| curve
 *
 * Input pins use pin-change interrupts to measure pulse width.
 * Output pins use analogWrite() (Timer-backed PWM, 8-bit, 490 Hz).
 *
 * Target: Arduino Uno / Nano (ATmega328P)
 *
 * Input  pins : 8  (Drive),  9 (Turn), 10 (Weapon)
 * Output pins : 3  (Drive),  5 (Turn),  6 (Weapon)
 *
 * Standard RC PWM pulse range: 1000 µs (full reverse/off) – 2000 µs (full forward/on)
 * Neutral / center:            1500 µs
 */

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Gyro-compensation configuration
// ---------------------------------------------------------------------------

// Select scaling law:
//   0 = LINEAR    omega = omega0 * (1 - GYRO_K * |Omega|)
//   1 = HYPERBOLIC  omega = min(omega0, GYRO_TAU_LIMIT / |Omega|)
#define WEAPON_SCALE_MODE 0

// LINEAR mode — tuning constant k (0.0–1.0).
// Recommended starting point: 0.4  (reduce if too aggressive, increase if bot still wheelies)
// At k=0.4:  |Omega|=0 -> 100%, |Omega|=0.25 -> 90%, |Omega|=0.5 -> 80%, |Omega|=1.0 -> 60%
constexpr float GYRO_K = 0.4f;

// HYPERBOLIC mode — normalised torque limit  tau_max / I  (0.0–1.0].
// Smaller values = more aggressive reduction at moderate turns.
// Typical starting point: 0.5
// At 0.5:  |Omega|=0.25 -> 100% (clamped), |Omega|=0.5 -> 100% (clamped), |Omega|=1.0 -> 50%
constexpr float GYRO_TAU_LIMIT = 0.5f;

// RC PWM timing constants (microseconds)
constexpr uint16_t PWM_MIN     = 1000;
constexpr uint16_t PWM_MAX     = 2000;
constexpr uint16_t PWM_NEUTRAL = 1500;

// Timeout: if no valid pulse received within this window, treat as neutral.
constexpr uint32_t SIGNAL_TIMEOUT_MS = 100;

// Input pins (must support pin-change interrupts on ATmega328P)
constexpr uint8_t PIN_IN_DRIVE  = 8;   // PCINT0 (PCMSK0, bit 0)
constexpr uint8_t PIN_IN_TURN   = 9;   // PCINT1 (PCMSK0, bit 1)
constexpr uint8_t PIN_IN_WEAPON = 10;  // PCINT2 (PCMSK0, bit 2)

// Output pins (must be PWM-capable via analogWrite)
constexpr uint8_t PIN_OUT_DRIVE  = 3;
constexpr uint8_t PIN_OUT_TURN   = 5;
constexpr uint8_t PIN_OUT_WEAPON = 6;

// ---------------------------------------------------------------------------
// Shared state updated by ISR
// ---------------------------------------------------------------------------

struct Channel {
    volatile uint32_t pulseStart  = 0;   // micros() when rising edge seen
    volatile uint16_t pulseWidth  = PWM_NEUTRAL; // last measured pulse (µs)
    volatile uint32_t lastUpdate  = 0;   // millis() of last good pulse
    volatile bool     risingEdge  = false;
};

static Channel chDrive;
static Channel chTurn;
static Channel chWeapon;

// ---------------------------------------------------------------------------
// Pin-Change Interrupt — PCINT0 (handles pins D8–D13)
// ---------------------------------------------------------------------------

ISR(PCINT0_vect) {
    uint32_t now = micros();
    uint8_t  pinState = PINB; // Read port B snapshot atomically

    // --- Drive (PB0 = D8) ---
    bool drivePinHigh = pinState & (1 << PB0);
    if (drivePinHigh && !chDrive.risingEdge) {
        chDrive.pulseStart  = now;
        chDrive.risingEdge  = true;
    } else if (!drivePinHigh && chDrive.risingEdge) {
        uint16_t width = (uint16_t)(now - chDrive.pulseStart);
        if (width >= 800 && width <= 2200) {          // sanity-check range
            chDrive.pulseWidth = width;
            chDrive.lastUpdate = millis();
        }
        chDrive.risingEdge = false;
    }

    // --- Turn (PB1 = D9) ---
    bool turnPinHigh = pinState & (1 << PB1);
    if (turnPinHigh && !chTurn.risingEdge) {
        chTurn.pulseStart  = now;
        chTurn.risingEdge  = true;
    } else if (!turnPinHigh && chTurn.risingEdge) {
        uint16_t width = (uint16_t)(now - chTurn.pulseStart);
        if (width >= 800 && width <= 2200) {
            chTurn.pulseWidth = width;
            chTurn.lastUpdate = millis();
        }
        chTurn.risingEdge = false;
    }

    // --- Weapon (PB2 = D10) ---
    bool weaponPinHigh = pinState & (1 << PB2);
    if (weaponPinHigh && !chWeapon.risingEdge) {
        chWeapon.pulseStart  = now;
        chWeapon.risingEdge  = true;
    } else if (!weaponPinHigh && chWeapon.risingEdge) {
        uint16_t width = (uint16_t)(now - chWeapon.pulseStart);
        if (width >= 800 && width <= 2200) {
            chWeapon.pulseWidth = width;
            chWeapon.lastUpdate = millis();
        }
        chWeapon.risingEdge = false;
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/**
 * Safely read a channel's pulse width with interrupts briefly disabled.
 * Falls back to neutral if the signal has timed out.
 */
uint16_t safeRead(const Channel& ch) {
    uint16_t width;
    uint32_t lastUpdate;

    noInterrupts();
    width      = ch.pulseWidth;
    lastUpdate = ch.lastUpdate;
    interrupts();

    if ((millis() - lastUpdate) > SIGNAL_TIMEOUT_MS) {
        return PWM_NEUTRAL;  // lost signal → safe neutral
    }
    return width;
}

/**
 * Convert a raw RC pulse width (µs) directly to a PWM duty cycle (0–255)
 * with no scaling applied. Used for Drive and Turn outputs.
 */
uint8_t pulseToOutput(uint16_t inputUs) {
    inputUs = constrain(inputUs, PWM_MIN, PWM_MAX);
    return (uint8_t)map(inputUs, PWM_MIN, PWM_MAX, 0, 255);
}

/**
 * Gyro-compensated weapon scaling.
 *
 * weaponUs : Weapon channel pulse width (µs), 1000–2000
 * turnUs   : Turn   channel pulse width (µs), 1000–2000
 *
 * Both channels treat 1500 µs as neutral.  The weapon channel is assumed to be
 * a unidirectional ESC (1500 = off, 2000 = full speed), so only the positive
 * deflection matters; the sign is preserved for bidirectional ESCs automatically.
 *
 * Returns analogWrite-compatible duty cycle 0–255.
 */
uint8_t scaleWeaponByTurn(uint16_t weaponUs, uint16_t turnUs) {
    weaponUs = constrain(weaponUs, PWM_MIN, PWM_MAX);
    turnUs   = constrain(turnUs,   PWM_MIN, PWM_MAX);

    // Normalised turn rate |Omega|: 0.0 = straight, 1.0 = full lock
    float absOmega = fabsf((float)turnUs - (float)PWM_NEUTRAL) / 500.0f;

    // Normalised weapon command omega0: 0.0 = off, 1.0 = full speed
    // (deflection from neutral, preserves sign for bidirectional ESCs)
    float weaponDeflection = ((float)weaponUs - (float)PWM_NEUTRAL) / 500.0f;

    float scaledDeflection;

#if WEAPON_SCALE_MODE == 0
    // --- LINEAR scaling -------------------------------------------
    // omega = omega0 * (1 - k * |Omega|)
    // Simple, smooth, and easy to tune on the field.
    float weaponScale = 1.0f - GYRO_K * absOmega;
    weaponScale       = constrain(weaponScale, 0.0f, 1.0f);
    scaledDeflection  = weaponDeflection * weaponScale;

#else
    // --- HYPERBOLIC (physics-based) scaling -----------------------
    // Holds  tau = I * omega * Omega <= tau_max
    // Solved: omega <= tau_max/I / |Omega|  = GYRO_TAU_LIMIT / |Omega|
    // omega0 is the upper bound (driver's requested speed).
    float maxDeflection;
    if (absOmega < 1e-3f) {
        // Essentially straight — no reduction needed
        maxDeflection = 1.0f;
    } else {
        maxDeflection = GYRO_TAU_LIMIT / absOmega;   // 1/|Omega| curve
    }
    maxDeflection    = constrain(maxDeflection, 0.0f, 1.0f);
    // Allow up to omega0 but never more than the torque limit allows
    float effectiveDeflection = fabsf(weaponDeflection);
    effectiveDeflection       = fminf(effectiveDeflection, maxDeflection);
    // Re-apply original sign
    scaledDeflection = (weaponDeflection >= 0.0f) ? effectiveDeflection : -effectiveDeflection;
#endif

    // Reconstruct pulse width and map to 0–255
    int16_t scaledUs = (int16_t)PWM_NEUTRAL + (int16_t)(scaledDeflection * 500.0f);
    scaledUs         = constrain(scaledUs, (int16_t)PWM_MIN, (int16_t)PWM_MAX);
    return (uint8_t)map(scaledUs, PWM_MIN, PWM_MAX, 0, 255);
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);

    // Configure input pins
    pinMode(PIN_IN_DRIVE,  INPUT);
    pinMode(PIN_IN_TURN,   INPUT);
    pinMode(PIN_IN_WEAPON, INPUT);

    // Configure output pins
    pinMode(PIN_OUT_DRIVE,  OUTPUT);
    pinMode(PIN_OUT_TURN,   OUTPUT);
    pinMode(PIN_OUT_WEAPON, OUTPUT);

    // Enable Pin-Change Interrupts on PCINT0 group (pins D8–D10 = PB0–PB2)
    PCICR  |= (1 << PCIE0);                           // enable PCINT0..7 group
    PCMSK0 |= (1 << PCINT0) | (1 << PCINT1) | (1 << PCINT2); // mask pins 8,9,10

    Serial.println(F("PWM passthrough ready — gyro compensation active."));
#if WEAPON_SCALE_MODE == 0
    Serial.print(F("Mode: LINEAR  k="));
    Serial.println(GYRO_K);
#else
    Serial.print(F("Mode: HYPERBOLIC  tau_limit/I="));
    Serial.println(GYRO_TAU_LIMIT);
#endif
}

void loop() {
    // Read inputs (interrupt-safe)
    uint16_t rawDrive  = safeRead(chDrive);
    uint16_t rawTurn   = safeRead(chTurn);
    uint16_t rawWeapon = safeRead(chWeapon);

    // Drive and Turn pass through unchanged; Weapon is scaled by turn intensity
    analogWrite(PIN_OUT_DRIVE,  pulseToOutput(rawDrive));
    analogWrite(PIN_OUT_TURN,   pulseToOutput(rawTurn));
    analogWrite(PIN_OUT_WEAPON, scaleWeaponByTurn(rawWeapon, rawTurn));

    // Debug output — remove or comment out in production
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 100) {
        lastPrint = millis();
        float absOmega = fabsf((float)rawTurn - 1500.0f) / 500.0f;
#if WEAPON_SCALE_MODE == 0
        float wScale = constrain(1.0f - GYRO_K * absOmega, 0.0f, 1.0f);
#else
        float wScale = (absOmega < 1e-3f) ? 1.0f : constrain(GYRO_TAU_LIMIT / absOmega, 0.0f, 1.0f);
#endif
        Serial.print(F("Drive: "));    Serial.print(rawDrive);
        Serial.print(F("us->"));       Serial.print(pulseToOutput(rawDrive));
        Serial.print(F("  Turn: "));   Serial.print(rawTurn);
        Serial.print(F("us |Omega|=")); Serial.print(absOmega, 2);
        Serial.print(F("  Weapon: ")); Serial.print(rawWeapon);
        Serial.print(F("us scale="));  Serial.print(wScale, 2);
        Serial.print(F("->"));         Serial.println(scaleWeaponByTurn(rawWeapon, rawTurn));
    }

    // Small yield; the ISR handles timing-critical work
    delay(10);
}