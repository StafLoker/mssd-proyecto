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

// == Pins ==

#define BUTTON_PIN 2
#define SENSOR_PIN 3
#define DRIVER_ENABLE_PIN 6  // PWM
#define DRIVER_DIR_A_PIN 7
#define DRIVER_DIR_B_PIN 8

// == Fase 1 ==
#define DRIVER_OUT_A_PIN A0
#define DRIVER_OUT_B_PIN A1

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
};

Motor motor(DRIVER_ENABLE_PIN, DRIVER_DIR_A_PIN, DRIVER_DIR_B_PIN);

void setup() {
  // Modelo
  pinMode(BUTTON_PIN, INPUT); // No lo usamos en Fase 1

  motor.init();

  Serial.begin(9600);
}

void loop() {
  Serial.print("Driver OUT A: ");
  Serial.println((float) analogRead(DRIVER_OUT_A_PIN) * 0.00488); // 5.0 V / 1024.0 (ADC 10 bit precision -> 2^10 = 1024 codigos)

  Serial.print("Driver OUT B: ")
  Serial.println((float) analogRead(DRIVER_OUT_B_PIN) * 0.00488);
  delay(50);
}
