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
#include <PID_AutoTune_v0.h>

#define HALL_A_PIN 8
#define HALL_B_PIN 7
#define HALL_C_PIN 6

#define SPEED_UP A0
#define SPEED_DOWN A1
// #define BAT_VOLTAGE_PIN A0
// #define R_SHUNT_PIN A1
// #define THROTTLE_PIN A2
// #define TEMP_PIN A3

#define PWM_MAX_DUTY 255
#define PWM_MIN_DUTY 0
#define PWM_START_DUTY 100

volatile byte hall_state = 0;
byte motor_speed;

void setup() {
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

  pinMode(SPEED_UP, INPUT_PULLUP);
  pinMode(SPEED_DOWN, INPUT_PULLUP);

  SET_PWM_DUTY(PWM_START_DUTY);
  motor_speed = PWM_START_DUTY;

  sei();          // Enable global interrupts
  UPDATE_HALL();  // Initialize state
}

void loop() {
  // Speed adjustment
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
  
  // motor_speed = map(analogRead(THROTTLE_PIN), 0, 1024, 0, 255);
  // SET_PWM_DUTY(motor_speed);
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

  // Read hall sensor bits
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

// unsigned long prev = 0;
// void MEASUREMENT() {
//   if (millis() - prev > 1000) {
//     float V_shunt = analogRead(R_SHUNT_PIN) * (5.0 / 1024.0);
//     float current = V_shunt * (30.0 / 0.075);
//     Serial.print("Current: ");
//     Serial.println(current);

//     float V_bat = analogRead(BAT_VOLTAGE_PIN) * (5.0 / 1024.0) * 12.0;
//     float temp_sig = analogRead(TEMP_PIN);

//     if (V_bat < 48.0) {
//       Serial.println("ALERT: LOW BATTERY");
//     }
//     if (temp_sig > 100.0) {
//       Serial.println("ALERT: HIGH TEMPERATURE");
//     }
//     prev = millis();
//   }
// }
