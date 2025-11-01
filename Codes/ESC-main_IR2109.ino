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
#include <Adafruit_PCF8574.h>
#include <OneWire.h>
#include <DallasTemperature.h>
// #include <AutoPID.h>

//PCF8574 PNIS
#define POWERON_ALERT_PIN 0
#define LOWBAT_ALERT_PIN 1
#define HIGHTEMP_ALERT_PIN 2
#define SENSORLESS_MODE_PIN 7
#define CONTROL_MODE_PIN 6
#define DEBUG_MODE_PIN 5
#define REVERSE_MODE_PIN 4

//Arduino PINS
#define BRAKE_PIN 2
#define HALL_A_PIN 8
#define HALL_B_PIN 7
#define HALL_C_PIN 6

#define THROTTLE_PIN A2
#define TEMP_PIN A3
#define ONE_WIRE_BUS A3
#define BAT_VOLTAGE_PIN A6

#define PWM_MAX_DUTY 230
#define PWM_MIN_DUTY 0
#define PWM_START_DUTY 25

// PID control
// #define Kp 0.013
// #define Ki 16.61
// #define Kd 0

volatile byte hall_state = 0, prev_hall_state = 0;
volatile uint16_t int_count = 0;
double set_motor_speed = 300;  //RPM
double PWM_DUTY = 0.10 * 255.0;
bool SENSORLESS_MODE = false, CONTROL_MODE = false, DEBUG_MODE = false, REVERSE_MODE = false;
volatile byte brake_on = 0;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature temp(&oneWire);
Adafruit_PCF8574 pcf;
// AutoPID myPID(&meas_motor_speed, &set_motor_speed, &PWM_DUTY, PWM_MIN_DUTY, PWM_MAX_DUTY, Kp, Ki, Kd);

void setup() {
  Serial.begin(115200);
  Serial.println("INITIALIZING...");
  temp.begin();
  if (!pcf.begin(0x20, &Wire)) {
    Serial.println("PCF8574 ERROR");
    while (1) {}
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

  // Enable interrupt for pin D2
  attachInterrupt(digitalPinToInterrupt(BRAKE_PIN), UPDATE_BRAKE, CHANGE);

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

  TCCR0A = 0;
  // Timer1 Phase Correct 8-bit
  TCCR1A = 0;  // Timer off initially
  TCCR2A = 0;
  TCCR1A = (1 << WGM10);
  TCCR1B = (1 << CS10);  // prescaler = 1

  // Timer2 Phase Correct
  TCCR2A = (1 << WGM20);
  TCCR2B = (1 << CS20);  // prescaler = 1

  if (pcf.digitalRead(SENSORLESS_MODE_PIN) == LOW) {
    SENSORLESS_MODE = true;
    Serial.println("SENSORLESS MODE ON");
  }
  if (pcf.digitalRead(CONTROL_MODE_PIN) == LOW) {
    CONTROL_MODE = true;
    Serial.println("TEST MODE ON");
    SET_PWM_DUTY(PWM_DUTY);
  } else {
    SET_PWM_DUTY(0);
  }
  if (pcf.digitalRead(DEBUG_MODE_PIN) == LOW) {
    DEBUG_MODE = true;
    Serial.println("DEBUG MODE ON");
  }
  if (pcf.digitalRead(REVERSE_MODE_PIN) == LOW) {
    REVERSE_MODE = true;
    Serial.println("REVERSE MODE ON");
  }
  // float tol = set_motor_speed * 0.03;  //3% band
  // myPID.atSetPoint(tol);
  // myPID.setBangBang(4);
  // myPID.setTimeStep(4000);

  sei();          // Enable global interrupts
  UPDATE_HALL();  // Initialize state
  BLDC_MOVE();
  pcf.digitalWrite(POWERON_ALERT_PIN, HIGH);
}

void loop() {
  if (CONTROL_MODE) {
    //Constant speed control mode
    // myPID.run();
    SET_PWM_DUTY(PWM_DUTY);
  } else {
    //eScooter control mode
    double throttle = analogRead(THROTTLE_PIN);
    PWM_DUTY = map(throttle, 184, 1023, 0, 255);
    PWM_DUTY = constrain(PWM_DUTY, PWM_MIN_DUTY, PWM_MAX_DUTY);
    SET_PWM_DUTY(PWM_DUTY);
    // Serial.println(throttle);
  }

  GET_MOTOR_SPEED();
  MEASUREMENT();
  DEBUG();
}

ISR(PCINT0_vect) {
  // Interrupt for PORTB pin (D8)
  UPDATE_HALL();
  BLDC_MOVE();
}

ISR(PCINT2_vect) {
  // Interrupt for PORTD pins (D6 and D7)
  UPDATE_HALL();
  BLDC_MOVE();
}

void UPDATE_BRAKE() {
  brake_on = (PIND & 0b00000100) ? 1 : 0;  // Check bit 2 (pin 2)
}

void UPDATE_HALL() {
  uint8_t pinb = PINB;  // Read PINB
  uint8_t pind = PIND;  // Read PIND

  // brake_on = (pind & 0b00000100) ? 1 : 0;               // Check bit 2 (pin 2)
  byte hall_a = (pinb & 0b00000001) ? 1 : 0;            // Check bit 0 (pin 8)
  byte hall_b = (pind & 0b10000000) ? 1 : 0;            // Check bit 7 (pin 7)
  byte hall_c = (pind & 0b01000000) ? 1 : 0;            // Check bit 6 (pin 6)
  hall_state = (hall_c << 2) | (hall_b << 1) | hall_a;  // CBA
  if (hall_state != prev_hall_state) {
    int_count++;
    prev_hall_state = hall_state;
  }
}

void BLDC_MOVE() {
  brake_on = (PIND & 0b00000100) ? 1 : 0;  // Check bit 2 (pin 2)
  if (!brake_on) {
    if (REVERSE_MODE) {
      switch (hall_state) {
        case 0b101: BH_AL(); break;  // B high, C low
        case 0b100: CH_AL(); break;  // A high, C low
        case 0b110: CH_BL(); break;  // A high, B low
        case 0b010: AH_BL(); break;  // C high, B low
        case 0b011: AH_CL(); break;  // C high, A low
        case 0b001: BH_CL(); break;  // B high, A low
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

// Enable PWM outputs (SET PWM)
// TCCR1A |= (1 << COM1A1); // Enable PWM IN(C) on OC1A D9
// TCCR1A |= (1 << COM1B1); // Enable PWM IN(B) on OC1B D10
// TCCR2A |= (1 << COM2A1); // Enable PWM IN(A) on OC2A D11

// Disable PWM outputs (RESET PWM)
// TCCR1A &= ~(1 << COM1A1); // Disable PWM IN(C) on OC1A D9
// TCCR1A &= ~(1 << COM1B1); // Disable PWM IN(B) on OC1B D10
// TCCR2A &= ~(1 << COM2A1); // Disable PWM IN(A) on OC2A D11

// SET IN
// PORTB |= (1 << PB1);      // HIGH IN(C) on D9
// PORTB |= (1 << PB2);      // HIGH IN(B) on D10
// PORTB |= (1 << PB3);      // HIGH IN(A) on D11

// RESET IN
// PORTB &= ~(1 << PB1);      // LOW IN(C) on D9
// PORTB &= ~(1 << PB2);      // LOW IN(B) on D10
// PORTB &= ~(1 << PB3);      // LOW IN(A) on D11

// SET ~SD
// PORTD |= (1 << PD3);  // HIGH ~SD(C) on D3
// PORTD |= (1 << PD4);  // HIGH ~SD(B) on D4
// PORTD |= (1 << PD5);  // HIGH ~SD(A) on D5

// RESET ~SD
// PORTD &= ~(1 << PD3);      // LOW ~SD(C) on D3
// PORTD &= ~(1 << PD4);      // LOW ~SD(B) on D4
// PORTD &= ~(1 << PD5);      // LOW ~SD(A) on D5

void all_off() {
  PORTD &= ~(1 << PD3);      // LOW ~SD(C) on D3
  PORTD &= ~(1 << PD4);      // LOW ~SD(B) on D4
  PORTD &= ~(1 << PD5);      // LOW ~SD(A) on D5
  TCCR1A &= ~(1 << COM1A1);  // Disable PWM IN(C) on OC1A D9
  TCCR1A &= ~(1 << COM1B1);  // Disable PWM IN(B) on OC1B D10
  TCCR2A &= ~(1 << COM2A1);  // Disable PWM IN(A) on OC2A D11
}

void AH_BL() {
  PORTD &= ~(1 << PD3);      // LOW ~SD(C) on D3
  PORTB &= ~(1 << PB1);      // LOW IN(C) on D9
  PORTB &= ~(1 << PB2);      // LOW IN(B) on D10
  TCCR1A &= ~(1 << COM1A1);  // Disable PWM IN(C) on OC1A D9
  TCCR1A &= ~(1 << COM1B1);  // Disable PWM IN(B) on OC1B D10

  PORTB &= ~(1 << PB2);  // LOW IN(B) on D10
  PORTD |= (1 << PD4);   // HIGH ~SD(B) on D4

  PORTD |= (1 << PD5);      // HIGH ~SD(A) on D5
  TCCR2A |= (1 << COM2A1);  // Enable PWM IN(A) on OC2A D11
}

void AH_CL() {
  PORTD &= ~(1 << PD4);      // LOW ~SD(B) on D4
  PORTB &= ~(1 << PB1);      // LOW IN(C) on D9
  PORTB &= ~(1 << PB2);      // LOW IN(B) on D10
  TCCR1A &= ~(1 << COM1A1);  // Disable PWM IN(C) on OC1A D9
  TCCR1A &= ~(1 << COM1B1);  // Disable PWM IN(B) on OC1B D10

  PORTB &= ~(1 << PB1);  // LOW IN(C) on D9
  PORTD |= (1 << PD3);   // HIGH ~SD(C) on D3

  PORTD |= (1 << PD5);      // HIGH ~SD(A) on D5
  TCCR2A |= (1 << COM2A1);  // Enable PWM IN(A) on OC2A D11
}

void BH_CL() {
  PORTD &= ~(1 << PD5);      // LOW ~SD(A) on D5
  PORTB &= ~(1 << PB1);      // LOW IN(C) on D9
  PORTB &= ~(1 << PB3);      // LOW IN(A) on D11
  TCCR1A &= ~(1 << COM1A1);  // Disable PWM IN(C) on OC1A D9
  TCCR2A &= ~(1 << COM2A1);  // Disable PWM IN(A) on OC2A D11

  PORTB &= ~(1 << PB1);  // LOW IN(C) on D9
  PORTD |= (1 << PD3);   // HIGH ~SD(C) on D3

  PORTD |= (1 << PD4);      // HIGH ~SD(B) on D4
  TCCR1A |= (1 << COM1B1);  // Enable PWM IN(B) on OC1B D10
}

void BH_AL() {
  PORTD &= ~(1 << PD3);      // LOW ~SD(C) on D3
  PORTB &= ~(1 << PB1);      // LOW IN(C) on D9
  PORTB &= ~(1 << PB3);      // LOW IN(A) on D11
  TCCR1A &= ~(1 << COM1A1);  // Disable PWM IN(C) on OC1A D9
  TCCR2A &= ~(1 << COM2A1);  // Disable PWM IN(A) on OC2A D11

  PORTB &= ~(1 << PB3);  // LOW IN(A) on D11
  PORTD |= (1 << PD5);   // HIGH ~SD(A) on D5

  PORTD |= (1 << PD4);      // HIGH ~SD(B) on D4
  TCCR1A |= (1 << COM1B1);  // Enable PWM IN(B) on OC1B D10
}

void CH_AL() {
  PORTD &= ~(1 << PD4);      // LOW ~SD(B) on D4
  PORTB &= ~(1 << PB2);      // LOW IN(B) on D10
  PORTB &= ~(1 << PB3);      // LOW IN(A) on D11
  TCCR1A &= ~(1 << COM1B1);  // Disable PWM IN(B) on OC1B D10
  TCCR2A &= ~(1 << COM2A1);  // Disable PWM IN(A) on OC2A D11

  PORTB &= ~(1 << PB3);  // LOW IN(A) on D11
  PORTD |= (1 << PD5);   // HIGH ~SD(A) on D5

  PORTD |= (1 << PD3);      // HIGH ~SD(C) on D3
  TCCR1A |= (1 << COM1A1);  // Enable PWM IN(C) on OC1A D9
}

void CH_BL() {
  PORTD &= ~(1 << PD5);      // LOW ~SD(A) on D5
  PORTB &= ~(1 << PB2);      // LOW IN(B) on D10
  PORTB &= ~(1 << PB3);      // LOW IN(A) on D11
  TCCR1A &= ~(1 << COM1B1);  // Disable PWM IN(B) on OC1B D10
  TCCR2A &= ~(1 << COM2A1);  // Disable PWM IN(A) on OC2A D11

  PORTB &= ~(1 << PB2);  // LOW IN(B) on D10
  PORTD |= (1 << PD4);   // HIGH ~SD(B) on D4

  PORTD |= (1 << PD3);      // HIGH ~SD(C) on D3
  TCCR1A |= (1 << COM1A1);  // Enable PWM IN(C) on OC1A D9
}

void SET_PWM_DUTY(byte duty) {
  duty = constrain(duty, PWM_MIN_DUTY, PWM_MAX_DUTY);
  OCR1A = duty;  // Set PWM duty cycle of PD9 (CH)
  OCR1B = duty;  // Set PWM duty cycle of PD10 (BH)
  OCR2A = duty;  // Set PWM duty cycle of PD11 (AH)
}

const byte pole_pairs = 15;
unsigned long prev_rpm_time = 0;
double meas_motor_speed = 0;  //RPM
void GET_MOTOR_SPEED() {
  if (millis() - prev_rpm_time >= 1000) {
    uint16_t count = int_count;
    int_count = 0;
    meas_motor_speed = (60.0 * count) / (6.0 * pole_pairs);
    // Serial.print("Interrupt Count: ");
    // Serial.println(count);
    // Serial.print("RPM: ");
    // Serial.println(meas_motor_speed);
    prev_rpm_time = millis();
  }
}

unsigned long prev_measure_time = 0;
void MEASUREMENT() {
  float Vref = 5.0;
  float voltageRatio = 7.0;
  if (millis() - prev_measure_time > 1000) {
    float V_bat = analogRead(BAT_VOLTAGE_PIN) * (Vref / 1024.0) * voltageRatio;
    if (V_bat < 24.0) {
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

    Serial.print("Voltage: ");
    Serial.print(V_bat);
    // Serial.print(" V\t\tCurrent: ");
    // Serial.print(current);
    Serial.print(" A\tTemp: ");
    Serial.print(temp_c);
    Serial.println(" C");
    Serial.print("RPM: ");
    Serial.println(meas_motor_speed);
    Serial.println("------------------------------------------------------------------------------");
    prev_measure_time = millis();
  }
}

unsigned long prev_debug_time = 0;
void DEBUG() {
  if (DEBUG_MODE) {
    if (millis() - prev_debug_time > 10) {
      Serial.print("PWM DUTY: ");
      Serial.print(PWM_DUTY);
      Serial.print("\t");
      Serial.println((PWM_DUTY / 255) * 100);
      Serial.print("Hall(CBA): ");
      Serial.println(hall_state, BIN);
      Serial.print("RPM: ");
      Serial.println(meas_motor_speed);
      Serial.print("Brake: ");
      Serial.println(brake_on);
      Serial.println("------------------------------------------------------------------------------");
      prev_debug_time = millis();
    }
  }
}