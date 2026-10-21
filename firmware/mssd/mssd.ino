/**
* @author Stefan Oshchypok, Anthony Manzueta Difo
*
* Universidad Carlos III de Madrid
* Máster Universitario en Robótica y Automatización
* Asignatura: Modelizacion y Simulacion de Sistemas Dinamicos
* Curso: 2026 - 2027
*
* Componentes:
* - DHT11
* - Button (+ 10K Om resistencia)
* - L293D DC motor driver
* - DC motor con encoder
* - Power Supply Module
*/

#include <DHT.h>

// == Pins ==

#define BUTTON_PIN 5

#define SENSOR_PIN 4

#define DRIVER_ENABLE_PIN 6  // PWM
#define DRIVER_DIR_A_PIN 7
#define DRIVER_DIR_B_PIN 8

#define DRIVER_ENCODER_OUT_A 2
#define DRIVER_ENCODER_OUT_B 3

// == Other ==
#define SENSOR_TYPE DHT11

const float T_MIN = 20.0;
const float T_MAX = 30.0;

const float ENCODER_PPR = 20.0; // TODO: Sustituir por los pulsos por revolucion reales del encoder.
const unsigned long SENSOR_CAPTURE_INTERVAL_MS = 2000;
const unsigned long SPEED_CAPTURE_INTERVAL_MS = 1000;

enum class RotationDirection {
  FORWARD,
  INVERSE
};

class Motor {
private:
  int enable_pin_;
  int dir_a_pin_;
  int dir_b_pin_;

  int speed_;
  RotationDirection direction_;

public:
  static constexpr int STOP = 0;
  static constexpr int MAX_SPEED = 255;

  Motor(int enable_pin, int dir_a_pin, int dir_b_pin) {
    enable_pin_ = enable_pin;
    dir_a_pin_ = dir_a_pin;
    dir_b_pin_ = dir_b_pin;

    pinMode(enable_pin_, OUTPUT);
    pinMode(dir_a_pin_, OUTPUT);
    pinMode(dir_b_pin_, OUTPUT);

    speed_ = Motor::STOP;
    direction_ = RotationDirection::FORWARD;
  }

  void init() {
    change_speed(Motor::STOP);
    change_direction(RotationDirection::FORWARD);
  }

  int speed() const {
    return speed_;
  }

  RotationDirection direction() const {
    return direction_;
  }

  int change_speed(int speed) {
    if (speed < Motor::STOP || speed > Motor::MAX_SPEED)
      return 1;
    speed_ = speed;
    analogWrite(enable_pin_, speed_);
    return 0;
  }

  int change_direction(RotationDirection direction) {
    if (direction == RotationDirection::FORWARD) {
      direction_ = direction;
      digitalWrite(dir_a_pin_, HIGH);
      digitalWrite(dir_b_pin_, LOW);
    } else if (direction == RotationDirection::INVERSE) {
      direction_ = direction;
      digitalWrite(dir_a_pin_, LOW);
      digitalWrite(dir_b_pin_, HIGH);
    } else {
      return 1;
    }
    return 0;
  }

  int swap_direction() {
    if (direction_ == RotationDirection::FORWARD) {
      change_direction(RotationDirection::INVERSE);
    } else if (direction_ == RotationDirection::INVERSE) {
      change_direction(RotationDirection::FORWARD);
    } else {
      return 1;
    }
    return 0;
  }
};

// == Help functions ==

int temperatureToPWM(float temperature) {
  float output = (temperature - T_MIN)
                 / (T_MAX - T_MIN) * 255.0;

  return (int)constrain(output, 0, 255);
}

float pwmToVoltage(int pwm) {
  return pwm * (5.0 / 255.0);
}

float rpmToAngularVelocity(float rpm) {
  return rpm * (2.0 * PI / 60.0);
}

// == Global vars ==

DHT sensor(SENSOR_PIN, SENSOR_TYPE);
Motor motor(DRIVER_ENABLE_PIN, DRIVER_DIR_A_PIN, DRIVER_DIR_B_PIN);

volatile unsigned long encoderPulses = 0;

unsigned long lastSensorTime = 0;
unsigned long lastSpeedTime = 0;

int lastButtonState = LOW;

// == Interupciones ==

void encoderISR() {
  encoderPulses++;
}


void setup() {
  Serial.begin(9600);

  // Modelo
  pinMode(BUTTON_PIN, INPUT);
  pinMode(DRIVER_ENCODER_OUT_A, INPUT);
  pinMode(DRIVER_ENCODER_OUT_B, INPUT);

  sensor.begin();
  motor.init();

  attachInterrupt(
    digitalPinToInterrupt(DRIVER_ENCODER_OUT_A),
    encoderISR,
    RISING);
}

// Fase 1
void loop() {
  // == Run motor at maximum PWM ==

  motor.change_speed(Motor::MAX_SPEED);

  // == Measure angular velocity using encoder ==

  unsigned long now = millis();

  if (now - lastSpeedTime >= SPEED_CAPTURE_INTERVAL_MS) {
    unsigned long elapsedMs = now - lastSpeedTime;
    lastSpeedTime = now;

    noInterrupts();
    unsigned long pulses = encoderPulses;
    encoderPulses = 0;
    interrupts();

    float elapsedSeconds = elapsedMs / 1000.0;

    float rpm = 0.0;

    if (ENCODER_PPR > 0.0) {
      rpm = (pulses / ENCODER_PPR)
            * (60.0 / elapsedSeconds);
    }

    float omega = rpmToAngularVelocity(rpm);

    Serial.print("PWM: ");
    Serial.println(Motor::MAX_SPEED);

    Serial.print("Encoder pulses: ");
    Serial.println(pulses);

    Serial.print("Motor speed: ");
    Serial.print(rpm, 2);
    Serial.println(" RPM");

    Serial.print("Angular velocity: ");
    Serial.print(omega, 3);
    Serial.println(" rad/s");

    Serial.println();
  }
}

/* Fase 2
void loop() {
  unsigned long now = millis();

  // == Read temperature and control motor ==

  if (now - lastSensorTime >= SENSOR_CAPTURE_INTERVAL_MS) {
    lastSensorTime = now;

    float temperature = sensor.readTemperature();

    if (isnan(temperature)) {
      Serial.println("Error leyendo el sensor DHT11");
    } else {
      int pwm = temperatureToPWM(temperature);
      float voltage = pwmToVoltage(pwm);

      motor.change_speed(pwm);

      Serial.println("--- Motor control ---");

      Serial.print("Temperature: ");
      Serial.print(temperature, 1);
      Serial.println(" C");

      Serial.print("PWM: ");
      Serial.print(pwm);
      Serial.print(" / 255");

      Serial.print(" | Voltage equivalent: ");
      Serial.print(voltage, 2);
      Serial.println(" V");
    }
  }

  // == Toggle direction once per button press ==

  int buttonState = digitalRead(BUTTON_PIN);

  if (buttonState == HIGH && lastButtonState == LOW) {
    motor.swap_direction();

    Serial.print("Direction: ");
    Serial.println(
      motor.direction() == RotationDirection::FORWARD
        ? "FORWARD"
        : "INVERSE");
  }

  lastButtonState = buttonState;

  // == Measure angular velocity using encoder ==

  now = millis();

  if (now - lastSpeedTime >= SPEED_CAPTURE_INTERVAL_MS) {
    unsigned long elapsedMs = now - lastSpeedTime;
    lastSpeedTime = now;

    noInterrupts();
    unsigned long pulses = encoderPulses;
    encoderPulses = 0;
    interrupts();

    float elapsedSeconds = elapsedMs / 1000.0;

    float rpm = 0.0;

    if (ENCODER_PPR > 0.0) {
      rpm = (pulses / ENCODER_PPR)
            * (60.0 / elapsedSeconds);
    }

    float omega = rpmToAngularVelocity(rpm);

    Serial.print("Encoder pulses: ");
    Serial.println(pulses);

    Serial.print("Motor speed: ");
    Serial.print(rpm, 2);
    Serial.println(" RPM");

    Serial.print("Angular velocity: ");
    Serial.print(omega, 3);
    Serial.println(" rad/s");

    Serial.println();
  }
}
*/
