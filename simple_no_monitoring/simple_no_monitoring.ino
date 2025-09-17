#include <Arduino.h>
#include "driver/mcpwm.h"

// ----------------- CONFIG -----------------
const int PIN_MCPWM_A = 0;       // خروجی PWM A
const int PIN_MCPWM_B = 1;       // خروجی PWM B
const int LED_PIN = LED_BUILTIN; // RGB LED روی برد YD-ESP32-S3

const int CARRIER_FREQ = 20000;   // فرکانس PWM: 20kHz
const float DEAD_TIME_US = 2.0f;  // dead-time میکروثانیه
float duty = 50.0f;               // دیوتی شروع (٪)

// ----------------- FUNCTIONS -----------------
void mcpwmSetup() {
  // وصل کردن GPIOها به MCPWM
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_MCPWM_A);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PIN_MCPWM_B);

  // کانفیگ اولیه MCPWM
  mcpwm_config_t pwm_config;
  pwm_config.frequency = CARRIER_FREQ;
  pwm_config.cmpr_a = duty;          // دیوتی روی کانال A
  pwm_config.cmpr_b = 100.0 - duty;  // کانال B مکمل
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

  // dead-time
  uint32_t dead_ticks = (uint32_t)(DEAD_TIME_US * 80.0 + 0.5);
#ifdef CONFIG_IDF_TARGET_ESP32S3
  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0,
                        MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,
                        dead_ticks, dead_ticks);
#endif
}

void updateDuty(float newDuty) {
  duty = constrain(newDuty, 0.0f, 100.0f);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 100.0 - duty);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);

  Serial.printf("Duty updated: %.1f%%\n", duty);
}

// ----------------- SETUP -----------------
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  mcpwmSetup();

  Serial.println("DC PWM output ready (GPIO0=A, GPIO1=B)");
  Serial.println("Send: DUTY <value>   e.g. DUTY 30");
}

// ----------------- LOOP -----------------
void loop() {
  // LED چشمک‌زن برای Alive بودن
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);

  // بررسی دستورات سریال
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("DUTY")) {
      float newDuty = cmd.substring(5).toFloat();
      updateDuty(newDuty);
    }
  }
}
