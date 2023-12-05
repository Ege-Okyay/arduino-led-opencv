void setup() {
  // Initialize the LEDs
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

  // Initialize the serial communication
  Serial.begin(9600);
}

void loop() {
  // Check if there is data available on the serial port
  if (Serial.available()) {
    // Read the incoming data
    char receivedChar = Serial.read();

    // Control the LED based on the incoming data
    if (receivedChar == 'R') {
      digitalWrite(8, LOW);
      digitalWrite(9, HIGH);
    } else if (receivedChar == 'G') {
      digitalWrite(8, HIGH);
      digitalWrite(9, LOW);
    } else {
      digitalWrite(8, LOW);
      digitalWrite(9, LOW);
    }
  }
}