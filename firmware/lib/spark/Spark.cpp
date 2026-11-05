#include "Spark.h"
#include "CommandRouter.h"
#include "ec_pins.h"

constexpr uint32_t pwm_res = 16;         // 16 bit resolution
constexpr uint32_t spark_freq_hz = 50;   // 20 ms period
constexpr float spark_duty_cycle = 0.16; // 16% duty cycle results in 3.2 ms T_on

namespace Spark {

bool begin() {
  // turn off spark
  digitalWrite(PIN_SPARK_TRIG, LOW);
  pinMode(PIN_SPARK_TRIG, OUTPUT);

  // configure PWM settings
  analogWriteResolution(pwm_res);
  analogWriteFrequency(spark_freq_hz);

  // turn off PWM
  analogWrite(PIN_SPARK_PWM, 0);

  CommandRouter::add(spark_check, "spark_check", "Manually trigger spark for 100 ms.");

  return true;
}

void spark_check() {
  CommsSerial.println("Spark armed, triggering in 500 ms.");
  arm_spark();
  delay(500);

  CommsSerial.println("Spark on, dwelling for 100 ms.");
  spark_on();
  delay(100);
  spark_off();

  CommsSerial.println("Spark off, waiting for 500 ms.");
  delay(500);
  disarm_spark();

  CommsSerial.println("Spark disarmed, check complete.");
}

void arm_spark() {
  analogWrite(PIN_SPARK_PWM, spark_duty_cycle * ((1 << pwm_res) - 1));
}

void disarm_spark() {
  analogWrite(PIN_SPARK_PWM, 0);
}

void spark_on() {
  digitalWrite(PIN_SPARK_TRIG, HIGH);
}

void spark_off() {
  digitalWrite(PIN_SPARK_TRIG, LOW);
}

} // namespace Spark