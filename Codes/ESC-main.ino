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
#include <AutoPID.h>
#include <Adafruit_PCF8574.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//PCF8574 PNIS
#define POWERON_ALERT_PIN 0
#define LOWBAT_ALERT_PIN 1
#define HIGHTEMP_ALERT_PIN 2
#define SENSORLESS_MODE_PIN 4
#define CONTROL_MODE_PIN 5
#define DEBUG_MODE_PIN 6
#define REVERSE_MODE_PIN 7

//Arduino PINS
#define BRAKE_PIN 2
#define HALL_A_PIN 8
#define HALL_B_PIN 7
#define HALL_C_PIN 6

#define SPEED_UP 12
#define SPEED_DOWN 13

#define R_SHUNT_PIN A0
#define THROTTLE_PIN A2
#define TEMP_PIN A3
#define ONE_WIRE_BUS A3
#define BAT_VOLTAGE_PIN A6

#define PWM_MAX_DUTY 255
#define PWM_MIN_DUTY 0
#define PWM_START_DUTY 100

// PID control
#define Kp 0.013
#define Ki 16.61
#define Kd 0

volatile byte hall_state = 0;
volatile uint16_t hall_count = 0;
const byte pole_pairs = 7;  //6, 7, 14 pairs
unsigned long prev_rpm_time = 0;
double meas_motor_speed = 0;   //RPM
double set_motor_speed = 300;  //RPM
double PWM_DUTY = 100;
bool SENSORLESS_MODE = false, CONTROL_MODE = false, DEBUG_MODE = false, REVERSE_MODE = false;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature temp(&oneWire);
AutoPID myPID(&meas_motor_speed, &set_motor_speed, &PWM_DUTY, PWM_MIN_DUTY, PWM_MAX_DUTY, Kp, Ki, Kd);
Adafruit_PCF8574 pcf;

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
  pcf.pinMode(REVERSE_MODE_PIN, INPUT);

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

  TCCR1A = 0;     // Timer1 off initially
  TCCR1B = 0x01;  //No prescaler
  TCCR2A = 0;
  TCCR2B = 0x01;  //No prescaler

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
  if (pcf.digitalRead(REVERSE_MODE_PIN) == LOW) {
    REVERSE_MODE = true;
  }

  SET_PWM_DUTY(PWM_START_DUTY);

  float tol = set_motor_speed * 0.03;  //3% band
  myPID.atSetPoint(tol);
  // myPID.setBangBang(4);
  // myPID.setTimeStep(4000);

  sei();          // Enable global interrupts
  UPDATE_HALL();  // Initialize state
  pcf.digitalWrite(POWERON_ALERT_PIN, HIGH);
}

void loop() {
  GET_MOTOR_SPEED();
  MEASUREMENT();
  if (CONTROL_MODE) {
    //Constant speed control mode
    myPID.run();
    SET_PWM_DUTY(PWM_DUTY);
  } else {
    //eScooter control mode
    PWM_DUTY = map(analogRead(THROTTLE_PIN), 0, 1023, 0, 255);
    SET_PWM_DUTY(PWM_DUTY);
  }

  if (DEBUG_MODE) {
    Serial.print("PWM DUTY: ");
    Serial.println((PWM_DUTY / 255) * 100);
    Serial.print("Hall(CBA): ");
    Serial.println(hall_state, BIN);
    Serial.print("RPM: ");
    Serial.println(meas_motor_speed);
    Serial.println("------------------------------------------------------------------------------");
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
  hall_count++;
  uint8_t pinb = PINB;  // Read PINB
  uint8_t pind = PIND;  // Read PIND

  byte brake_on = (pind & 0b00000100) ? 1 : 0;  // Check bit 2 (pin 2)
  if (!brake_on) {
    byte hall_a = (pinb & 0b00000001) ? 1 : 0;  // Check bit 0 (pin 8)
    byte hall_b = (pind & 0b10000000) ? 1 : 0;  // Check bit 7 (pin 7)
    byte hall_c = (pind & 0b01000000) ? 1 : 0;  // Check bit 6 (pin 6)

    hall_state = (hall_c << 2) | (hall_b << 1) | hall_a;  // CBA
    if (REVERSE_MODE) {
      switch (hall_state) {
        case 0b101: BH_AL(); break;  // B high, A low
        case 0b100: CH_AL(); break;  // C high, A low
        case 0b110: CH_BL(); break;  // C high, B low
        case 0b010: AH_BL(); break;  // A high, B low
        case 0b011: AH_CL(); break;  // A high, C low
        case 0b001: BH_CL(); break;  // B high, C low
        default: all_off(); break;
      }
    } else {
      switch (hall_state) {
        case 0b101: BH_CL(); break;  // B high, C low
        case 0b100: AH_CL(); break;  // A high, C low
        case 0b110: AH_BL(); break;  // A high, B low
        case 0b010: CH_BL(); break;  // C high, B low
        case 0b011: CH_AL(); break;  // C high, A low
        case 0b001: BH_AL(); break;  // B high, A low
        default: all_off(); break;
      }
    }
  } else {
    all_off();  // Disable while braking
  }
}

void all_off() {
  TCCR1A = 0;            // Timer1 off
  TCCR2A = 0;            // Timer2 off
  PORTD &= ~0b00111000;  // Clear D3, D4, D5
}

void AH_BL() {
  PORTD &= ~0b00101000;  // Clear D3 (CL) and D5 (AL)
  PORTD |= 0b00010000;   // Set D4 (BL)
  TCCR1A = 0;            // Timer1 off
  TCCR2A = 0b10000001;   // Enable PWM on OC2A D11 (AH)
}

void AH_CL() {
  PORTD &= ~0b00110000;  // Clear D4 (BL), D5 (AL)
  PORTD |= 0b00001000;   // Set D3 (CL)
  TCCR1A = 0;            // Timer1 off
  TCCR2A = 0b10000001;   // Enable PWM on OC2A D11 (AH)
}

void BH_CL() {
  PORTD &= ~0b00110000;  // Clear D4 (BL), D5 (AL)
  PORTD |= 0b00001000;   // Set D3 (CL)
  TCCR2A = 0;            // Timer2 off
  TCCR1A = 0b00100001;   // Enable PWM on OC1B D10 (BH)
}

void BH_AL() {
  PORTD &= ~0b00011000;  // Clear D3 (CL), D4 (BL)
  PORTD |= 0b00100000;   // Set D5 (AL)
  TCCR2A = 0;            // Timer2 off
  TCCR1A = 0b00100001;   // Enable PWM on OC1B D10 (BH)
}

void CH_AL() {
  PORTD &= ~0b00011000;  // Clear D3 (CL), D4 (BL)
  PORTD |= 0b00100000;   // Set D5 (AL)
  TCCR2A = 0;            // Timer2 off
  TCCR1A = 0b10000001;   // Enable PWM on OC1A D9 (CH)
}

void CH_BL() {
  PORTD &= ~0b00101000;  // Clear D3 (CL), D5 (AL)
  PORTD |= 0b00010000;   // Set D4 (BL)
  TCCR2A = 0;            // Timer2 off
  TCCR1A = 0b10000001;   // Enable PWM on OC1A D9 (CH)
}

void SET_PWM_DUTY(byte duty) {
  duty = constrain(duty, PWM_MIN_DUTY, PWM_MAX_DUTY);
  OCR1A = duty;  // Set PWM duty cycle of PD9 (CH)
  OCR1B = duty;  // Set PWM duty cycle of PD10 (BH)
  OCR2A = duty;  // Set PWM duty cycle of PD11 (AH)
}

void GET_MOTOR_SPEED() {
  if (millis() - prev_rpm_time >= 100) {
    uint16_t count = hall_count;
    hall_count = 0;
    float mech_hz = (count * 10.0) / (6.0 * pole_pairs);
    meas_motor_speed = mech_hz * 60.0;
    prev_rpm_time = millis();
  }
}

unsigned long prev_time = 0;
void MEASUREMENT() {
  float Vref = 5.0;
  float voltageRatio = 7.0;
  if (millis() - prev_time > 1000) {
    float V_shunt = analogRead(R_SHUNT_PIN) * (Vref / 1024.0);
    float current = V_shunt * (20.0 / 0.075);

    float V_bat = analogRead(BAT_VOLTAGE_PIN) * (Vref / 1024.0) * voltageRatio;
    if (V_bat < 48.0) {
      Serial.println("ALERT: LOW BATTERY");
      pcf.digitalWrite(LOWBAT_ALERT_PIN, HIGH);
    } else {
      pcf.digitalWrite(LOWBAT_ALERT_PIN, LOW);
    }

    temp.requestTemperatures();
    float temp_c = temp.getTempCByIndex(0);
    if (temp_c > 100.0) {
      Serial.println("ALERT: HIGH TEMPERATURE");
      pcf.digitalWrite(HIGHTEMP_ALERT_PIN, HIGH);
    } else {
      pcf.digitalWrite(HIGHTEMP_ALERT_PIN, LOW);
    }

    if (DEBUG_MODE) {
      Serial.print("Voltage: ");
      Serial.print(V_bat);
      Serial.print(" V\tCurrent: ");
      Serial.println(current);
      Serial.print(" A\tTemp: ");
      Serial.print(temp_c);
      Serial.println(" C");
      Serial.print("RPM: ");
      Serial.println(meas_motor_speed);
      Serial.println("------------------------------------------------------------------------------");
    }
    prev_time = millis();
  }
}
