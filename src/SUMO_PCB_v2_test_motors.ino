/*
 * TC1508 Motor Driver Test for ESP32-C3 Super Mini
 * -------------------------------------------------
 * Drives 2 DC motors (Motor A and Motor B) through a TC1508 driver.
 *
 * Wiring (per your spec):
 *   ESP32-C3 GPIO  ->  TC1508 input
 *   ----------------------------------
 *   GPIO  9        ->  INT4   (Motor B direction B)
 *   GPIO 10        ->  INT3   (Motor B direction A)
 *   GPIO 20        ->  INT2   (Motor A direction B)
 *   GPIO 21        ->  INT1   (Motor A direction A)
 *
 * Truth table per motor (INT1/INT2 for Motor A, INT3/INT4 for Motor B):
 *   IN_A=LOW , IN_B=LOW  -> Coast / Stop
 *   IN_A=HIGH, IN_B=LOW  -> Forward
 *   IN_A=LOW , IN_B=HIGH -> Reverse
 *   IN_A=HIGH, IN_B=HIGH -> Brake
 *
 * *** IMPORTANT: GPIO 20 / 21 ARE UART0 (TX/RX) ***
 *   On the ESP32-C3 Super Mini, GPIO20 = UART0_TX and GPIO21 = UART0_RX.
 *   In Arduino-ESP32 core 2.x, calling Serial.begin(115200) re-assigns
 *   those pins to the UART hardware, which fights the TC1508's INT1/INT2.
 *   UART TX idles HIGH, RX idles LOW -> the motor on INT1/INT2 sees a
 *   constant REVERSE signal and spins nonstop. THIS IS WHY M2 (the motor
 *   on INT1/INT2) WAS SPINNING CONSTANTLY.
 *
 *   FIX OPTIONS:
 *     1) Leave logging disabled (default in this sketch). GPIO20/21 are
 *        free for PWM. No serial output, but motors behave correctly.
 *     2) To keep logging, enable "USB CDC On Boot" in the Arduino IDE:
 *          Tools -> USB CDC On Boot -> Enabled
 *        Then `Serial` goes over the native USB port and GPIO20/21 are
 *        released. Define USE_SERIAL below to re-enable the prints.
 *
 * PWM: LEDC peripheral, 8-bit resolution, 5 kHz, LEGACY API
 *   (ledcSetup / ledcAttachPin / ledcWrite) for Arduino-ESP32 core 2.x.
 *
 * EN pins of the TC1508 (if present) should be tied HIGH (3V3) or driven
 * by a PWM-capable pin; this sketch controls speed by PWM-ing the
 * direction pins directly, which works for the common TC1508 module.
 */

// --- Logging: define USE_SERIAL to enable Serial output -------------------
// On core 2.x, leaving this undefined frees GPIO20/21 from UART0 so they
// work as PWM outputs. See header comment for the USB-CDC alternative.
//#define USE_SERIAL

#include <Arduino.h>

#ifdef USE_SERIAL
  #define LOG_BEGIN()    Serial.begin(115200)
  #define LOG_PRINT(x)   Serial.print(x)
  #define LOG_PRINTLN(x) Serial.println(x)
#else
  #define LOG_BEGIN()    ((void)0)
  #define LOG_PRINT(x)   ((void)0)
  #define LOG_PRINTLN(x) ((void)0)
#endif

// ---- Pin map -------------------------------------------------------------
static const uint8_t PIN_INT1 = 21;  // Motor A IN1
static const uint8_t PIN_INT2 = 20;  // Motor A IN2
static const uint8_t PIN_INT3 = 10;  // Motor B IN3
static const uint8_t PIN_INT4 = 9;   // Motor B IN4

// ---- PWM configuration --------------------------------------------------
static const uint8_t  PWM_RESOLUTION = 8;          // 0..255
static const uint32_t PWM_FREQ_HZ    = 5000;       // 5 kHz

// LEDC channels (ESP32-C3 has 6 channels; we use 0..3)
static const uint8_t CH_INT1 = 0;
static const uint8_t CH_INT2 = 1;
static const uint8_t CH_INT3 = 2;
static const uint8_t CH_INT4 = 3;

// ---- Motor abstraction --------------------------------------------------
enum class Direction { STOP, FORWARD, REVERSE, BRAKE };

struct Motor {
  uint8_t inA_pin;   // direction A pin
  uint8_t inB_pin;   // direction B pin
  uint8_t inA_ch;    // LEDC channel for A
  uint8_t inB_ch;    // LEDC channel for B
};

Motor motorA = { PIN_INT1, PIN_INT2, CH_INT1, CH_INT2 };  // INT1/INT2
Motor motorB = { PIN_INT3, PIN_INT4, CH_INT3, CH_INT4 };  // INT3/INT4

// ---- Low-level helpers --------------------------------------------------
void writeMotor(const Motor &m, uint32_t dutyA, uint32_t dutyB) {
  ledcWrite(m.inA_ch, dutyA);
  ledcWrite(m.inB_ch, dutyB);
}

void driveMotor(const Motor &m, Direction dir, uint8_t speed = 255) {
  const uint32_t duty = map(speed, 0, 255, 0, (1 << PWM_RESOLUTION) - 1);
  switch (dir) {
    case Direction::STOP:    writeMotor(m, 0,   0);   break;  // coast
    case Direction::FORWARD: writeMotor(m, duty, 0);  break;
    case Direction::REVERSE: writeMotor(m, 0,   duty); break;
    case Direction::BRAKE:   writeMotor(m, duty, duty); break;  // short brake
  }
}

// ---- Test sequences ------------------------------------------------------
void testBothForward(uint8_t speed, uint32_t ms) {
  LOG_PRINTLN(">> Both motors FORWARD");
  driveMotor(motorA, Direction::FORWARD, speed);
  driveMotor(motorB, Direction::FORWARD, speed);
  delay(ms);
}

void testBothReverse(uint8_t speed, uint32_t ms) {
  LOG_PRINTLN(">> Both motors REVERSE");
  driveMotor(motorA, Direction::REVERSE, speed);
  driveMotor(motorB, Direction::REVERSE, speed);
  delay(ms);
}

void testSpin(uint8_t speed, uint32_t ms) {
  LOG_PRINTLN(">> Spin in place (A fwd, B rev)");
  driveMotor(motorA, Direction::FORWARD, speed);
  driveMotor(motorB, Direction::REVERSE, speed);
  delay(ms);
}

void testIndividual() {
  LOG_PRINTLN(">> Motor A only FORWARD");
  driveMotor(motorA, Direction::FORWARD, 200);
  driveMotor(motorB, Direction::STOP);
  delay(1500);

  LOG_PRINTLN(">> Motor B only FORWARD");
  driveMotor(motorA, Direction::STOP);
  driveMotor(motorB, Direction::FORWARD, 200);
  delay(1500);
}

void testRamp() {
  LOG_PRINTLN(">> Speed ramp 0 -> 255 -> 0 (both motors forward)");
  for (int s = 0; s <= 255; s += 5) {
    driveMotor(motorA, Direction::FORWARD, s);
    driveMotor(motorB, Direction::FORWARD, s);
    delay(30);
  }
  for (int s = 255; s >= 0; s -= 5) {
    driveMotor(motorA, Direction::FORWARD, s);
    driveMotor(motorB, Direction::FORWARD, s);
    delay(30);
  }
}

void testBrake(uint32_t ms) {
  LOG_PRINTLN(">> Brake (short both motors)");
  driveMotor(motorA, Direction::BRAKE, 255);
  driveMotor(motorB, Direction::BRAKE, 255);
  delay(ms);
}

void stopAll() {
  driveMotor(motorA, Direction::STOP);
  driveMotor(motorB, Direction::STOP);
}

// ---- Arduino setup / loop -----------------------------------------------
void setup() {
  LOG_BEGIN();
  delay(500);
  LOG_PRINTLN("");
  LOG_PRINTLN("=== TC1508 + ESP32-C3 Super Mini motor test ===");

  // Configure LEDC channels (legacy API: ledcSetup + ledcAttachPin)
  ledcSetup(CH_INT1, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcSetup(CH_INT2, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcSetup(CH_INT3, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcSetup(CH_INT4, PWM_FREQ_HZ, PWM_RESOLUTION);

  // Attach the direction pins to their LEDC channels
  ledcAttachPin(PIN_INT1, CH_INT1);
  ledcAttachPin(PIN_INT2, CH_INT2);
  ledcAttachPin(PIN_INT3, CH_INT3);
  ledcAttachPin(PIN_INT4, CH_INT4);

  // Make sure everything is off
  stopAll();
  delay(500);
}

void loop() {
  LOG_PRINTLN("");
  LOG_PRINTLN("--- New test cycle ---");

  testBothForward(180, 2000);
  testBrake(500);

  testBothReverse(180, 2000);
  testBrake(500);

  testSpin(180, 1500);
  testBrake(500);

  testIndividual();
  testBrake(500);

  testRamp();
  testBrake(800);

  LOG_PRINTLN("Coast 1.5 s");
  stopAll();
  delay(1500);
}
