#define HALL_A_PIN 8
#define HALL_B_PIN 7
#define HALL_C_PIN 6

volatile byte hall_state = 0, prev_hall_state = 0;
byte prev_hall_state1 = 0;
volatile uint16_t int_count = 0;
double meas_motor_speed = 0;  //RPM
unsigned long prev_rpm_time = 0;
const byte pole_pairs = 15;

void setup() {
  Serial.begin(115200);
  Serial.println("INITIALIZING...");
  delay(1000);

  pinMode(HALL_A_PIN, INPUT);
  pinMode(HALL_B_PIN, INPUT);
  pinMode(HALL_C_PIN, INPUT);

  // Enable PCINT for PORTB0 (pin 8)
  PCICR |= (1 << PCIE0);    // Enable PCINT0 vector (for PORTB)
  PCMSK0 |= (1 << PCINT0);  // Enable interrupt on PORTB0 (pin 8)

  // Enable PCINT for PORTD6 and PORTD7 (pins 6 and 7)
  PCICR |= (1 << PCIE2);                      // Enable PCINT2 vector (for PORTD)
  PCMSK2 |= (1 << PCINT22) | (1 << PCINT23);  // Enable interrupts on PORTD6 and PORTD7 (pins 6 and 7)

  sei();
  UPDATE_HALL();
}

void loop() {
  if (hall_state != prev_hall_state1) {
    Serial.print("Hall(CBA): ");
    Serial.println(hall_state, BIN);
    prev_hall_state1 = hall_state;
  }
  GET_MOTOR_SPEED();
}

ISR(PCINT0_vect) {
  // Interrupt for PORTB pin (D8)
  UPDATE_HALL();
}

ISR(PCINT2_vect) {
  // Interrupt for PORTD pins (D6 and D7)
  UPDATE_HALL();
}

void UPDATE_HALL() {
  uint8_t pinb = PINB;  // Read PINB
  uint8_t pind = PIND;  // Read PIND

  byte hall_a = (pinb & 0b00000001) ? 1 : 0;            // Check bit 0 (pin 8)
  byte hall_b = (pind & 0b10000000) ? 1 : 0;            // Check bit 7 (pin 7)
  byte hall_c = (pind & 0b01000000) ? 1 : 0;            // Check bit 6 (pin 6)
  hall_state = (hall_c << 2) | (hall_b << 1) | hall_a;  // CBA
  if (hall_state != prev_hall_state) {
    int_count++;
    prev_hall_state = hall_state;
  }
}

void GET_MOTOR_SPEED() {
  if (millis() - prev_rpm_time >= 1000) {
    uint16_t count = int_count;
    int_count = 0;
    meas_motor_speed = (60.0 * count) / (6.0 * pole_pairs);
    Serial.print("Interrupt Count: ");
    Serial.println(count);
    Serial.print("RPM: ");
    Serial.println(meas_motor_speed);
    Serial.println("------------------------------------------------------------------------------");
    prev_rpm_time = millis();
  }
}