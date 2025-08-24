/*MIT License

Copyright (c) 2025 peteatithep

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Arduino IDE 2.3.5
Arduino AVR Boards 1.8.6
*/
#include <Wire.h>
#include <PID_AutoTune_v0.h>
#include <Adafruit_PCF8574.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS A2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature temp(&oneWire);

Adafruit_PCF8574 pcf;
//PCF8574 PNIS
#define POWERON_ALERT_PIN 0
#define LOWBAT_ALERT_PIN 1
#define HIGHTEMP_ALERT_PIN 2
#define SENSORLESS_MODE_PIN 4
#define CONTROL_MODE_PIN 5
#define DEBUG_MODE_PIN 6
#define TEST_MODE_PIN 7

//Arduino PINS
#define BRAKE_PIN 2
#define HALL_A_PIN 8
#define HALL_B_PIN 7
#define HALL_C_PIN 6


#define SPEED_UP A1
#define SPEED_DOWN A7

#define R_SHUNT_PIN A0
#define THROTTLE_PIN A2
#define TEMP_PIN A3
#define BAT_VOLTAGE_PIN A6

#define PWM_MAX_DUTY 255
#define PWM_MIN_DUTY 0
#define PWM_START_DUTY 100

volatile byte hall_state = 0;
byte motor_speed;
bool SENSORLESS_MODE = false, CONTROL_MODE = false, DEBUG_MODE = false, TEST_MODE = false;

void setup() {
  Serial.begin(115200);
  Serial.println("INITIALIZING...");
  temp.begin();
  if (!pcf.begin(0x20, &Wire)) {
    Serial.println("PCF8574 ERROR");
    while (1);
  }
  pcf.pinMode(POWERON_ALERT_PIN, OUTPUT);
  pcf.pinMode(LOWBAT_ALERT_PIN, OUTPUT);
  pcf.pinMode(HIGHTEMP_ALERT_PIN, OUTPUT);
  pcf.pinMode(SENSORLESS_MODE_PIN, INPUT);
  pcf.pinMode(CONTROL_MODE_PIN, INPUT);
  pcf.pinMode(DEBUG_MODE_PIN, INPUT);
  pcf.pinMode(TEST_MODE_PIN, INPUT);

  pinMode(BRAKE_PIN, INPUT);
  pinMode(HALL_A_PIN, INPUT);
  pinMode(HALL_B_PIN, INPUT);
  pinMode(HALL_C_PIN, INPUT);

  // Enable PCINT for PORTB0 (pin 8)
  PCICR |= (1 << PCIE0);    // Enable PCINT0 vector (for PORTB)
  PCMSK0 |= (1 << PCINT0);  // Enable interrupt on PORTB0 (pin 8)

  // Enable PCINT for PORTD6 and PORTD7 (pins 6 and 7)
  PCICR |= (1 << PCIE2);                      // Enable PCINT2 vector (for PORTD)
  PCMSK2 |= (1 << PCINT22) | (1 << PCINT23);  // Enable interrupts on PORTD6 and PORTD7 (pins 6 and 7)

  DDRD |= 0b00111000;  // Enable pins 3, 4, 5 as outputs
  PORTD = 0x00;
  DDRB |= 0b00001110;  // Enable pins 9, 10, 11 as outputs
  PORTB = 0x00;

  TCCR1A = 0;  // Timer1 off initially
  TCCR1B = 0x01;
  TCCR2A = 0;
  TCCR2B = 0x01;

  if (pcf.digitalRead(SENSORLESS_MODE_PIN) == LOW) {
    SENSORLESS_MODE = true;
  }
  if (pcf.digitalRead(CONTROL_MODE_PIN) == LOW) {
    CONTROL_MODE = true;
    pinMode(SPEED_UP, INPUT_PULLUP);
    pinMode(SPEED_DOWN, INPUT_PULLUP);
  }
  if (pcf.digitalRead(DEBUG_MODE_PIN) == LOW) {
    DEBUG_MODE = true;
  }
  if (pcf.digitalRead(TEST_MODE_PIN) == LOW) {
    TEST_MODE = true;
  }

  SET_PWM_DUTY(PWM_START_DUTY);
  motor_speed = PWM_START_DUTY;

  sei();          // Enable global interrupts
  UPDATE_HALL();  // Initialize state
  pcf.digitalWrite(POWERON_ALERT_PIN, HIGH);
}

void loop() {
  if (CONTROL_MODE) {
    if (!digitalRead(SPEED_UP) && motor_speed < PWM_MAX_DUTY) {
      motor_speed++;
      SET_PWM_DUTY(motor_speed);
      delay(100);
    }
    if (!digitalRead(SPEED_DOWN) && motor_speed > PWM_MIN_DUTY) {
      motor_speed--;
      SET_PWM_DUTY(motor_speed);
      delay(100);
    }
  } else {
    motor_speed = map(analogRead(THROTTLE_PIN), 0, 1024, 0, 255);
    SET_PWM_DUTY(motor_speed);
  }

  if (DEBUG_MODE) {
    Serial.println((float(motor_speed) / 255) * 100);
    Serial.println(hall_state, BIN);
    Serial.println(motor_speed);
    delay(500);
  }
}

ISR(PCINT0_vect) {
  UPDATE_HALL();  // Interrupt for PORTB pins (pin 8)
}

ISR(PCINT2_vect) {
  UPDATE_HALL();  // Interrupt for PORTD pins (pins 6 and 7)
}

void UPDATE_HALL() {
  uint8_t pinb = PINB;  // Read PINB
  uint8_t pind = PIND;  // Read PIND

  byte brake_on = (pind & 0b00000100) ? 1 : 0;  // Check bit 0 (pin 8)
  if (!brake_on) {
    byte hall_a = (pinb & 0b00000001) ? 1 : 0;  // Check bit 0 (pin 8)
    byte hall_b = (pind & 0b10000000) ? 1 : 0;  // Check bit 7 (pin 7)
    byte hall_c = (pind & 0b01000000) ? 1 : 0;  // Check bit 6 (pin 6)

    hall_state = (hall_a << 2) | (hall_b << 1) | hall_c;
    switch (hall_state) {
      case 0b100: AH_BL(); break;
      case 0b101: AH_CL(); break;
      case 0b001: BH_CL(); break;
      case 0b011: BH_AL(); break;
      case 0b010: CH_AL(); break;
      case 0b110: CH_BL(); break;
      default: all_off(); break;  // Invalid state
    }
  } else {
    all_off();  // Disable while braking
  }
}

void all_off() {
  TCCR1A = 0;            // Timer1 off
  TCCR2A = 0;            // Timer2 off
  PORTD &= ~0b00111000;  // Clear PD3, PD4, PD5
}

void AH_BL() {
  PORTD &= ~0b00101000;  // Clear PD3 (CL) and PD5 (AL)
  PORTD |= 0b00010000;   // Set PD4 (BL)
  TCCR1A = 0;            // Timer1 off
  TCCR2A = 0b10000001;   // Enable PWM on OC2A PD11 (AH)
}

void AH_CL() {
  PORTD &= ~0b00110000;  // Clear PD4 (BL), PD5 (AL)
  PORTD |= 0b00001000;   // Set PD3 (CL)
  TCCR1A = 0;            // Timer1 off
  TCCR2A = 0b10000001;   // Enable PWM on OC2A PD11 (AH)
}

void BH_CL() {
  PORTD &= ~0b00110000;  // Clear PD4 (BL), PD5 (AL)
  PORTD |= 0b00001000;   // Set PD3 (CL)
  TCCR2A = 0;            // Timer2 off
  TCCR1A = 0b00100001;   // Enable PWM on OC1B PD10 (BH)
}

void BH_AL() {
  PORTD &= ~0b00011000;  // Clear PD3 (CL), PD4 (BL)
  PORTD |= 0b00100000;   // Set PD5 (AL)
  TCCR2A = 0;            // Timer2 off
  TCCR1A = 0b00100001;   // Enable PWM on OC1B PD10 (BH)
}

void CH_AL() {
  PORTD &= ~0b00011000;  // Clear PD3 (CL), PD4 (BL)
  PORTD |= 0b00100000;   // Set PD5 (AL)
  TCCR2A = 0;            // Timer2 off
  TCCR1A = 0b10000001;   // Enable PWM on OC1A PD9 (CH)
}

void CH_BL() {
  PORTD &= ~0b00101000;  // Clear PD3 (CL), PD5 (AL)
  PORTD |= 0b00010000;   // Set PD4 (BL)
  TCCR2A = 0;            // Timer2 off
  TCCR1A = 0b10000001;   // Enable PWM on OC1A PD9 (CH)
}

void SET_PWM_DUTY(byte duty) {
  duty = constrain(duty, PWM_MIN_DUTY, PWM_MAX_DUTY);
  OCR1A = duty;  // Set PWM duty cycle of PD9 (CH)
  OCR1B = duty;  // Set PWM duty cycle of PD10 (BH)
  OCR2A = duty;  // Set PWM duty cycle of PD11 (AH)
}

unsigned long prev = 0;
void MEASUREMENT() {
  float Vref = 5.0;
  float voltageRatio = 7.0;
  if (millis() - prev > 1000) {
    float V_shunt = analogRead(R_SHUNT_PIN) * (Vref / 1024.0);
    float current = V_shunt * (30.0 / 0.075);

    float V_bat = analogRead(BAT_VOLTAGE_PIN) * (Vref / 1024.0) * voltageRatio;
    if (V_bat < 48.0) {
      Serial.println("ALERT: LOW BATTERY");
      pcf.digitalWrite(LOWBAT_ALERT_PIN, HIGH);
    }

    temp.requestTemperatures();
    float temp_c = temp.getTempCByIndex(0);
    if (temp_c > 100.0) {
      Serial.println("ALERT: HIGH TEMPERATURE");
      pcf.digitalWrite(HIGHTEMP_ALERT_PIN, HIGH);
    }

    if (DEBUG_MODE) {
      Serial.print("Voltage: ");
      Serial.print(V_bat);
      Serial.print(" V\tCurrent: ");
      Serial.println(current);
      Serial.print(" A\tTemp: ");
      Serial.print(temp_c);
      Serial.println(" C");
    }
    prev = millis();
  }
}