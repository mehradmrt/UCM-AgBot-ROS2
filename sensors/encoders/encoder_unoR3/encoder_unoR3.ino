const int pin2 = 2;
const int pin3 = 3;

volatile int encoderValue = 0;
int lastEncoded = 0;

void setup() {
  Serial.begin(115200);

  // Make pin2 and pin3 as INPUT
  pinMode(pin2, INPUT);
  pinMode(pin3, INPUT);

  // Attach interrupts to pin2 and pin3
  attachInterrupt(digitalPinToInterrupt(pin2), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin3), updateEncoder, CHANGE);
}

void loop() {
  int initialEncoderValue = encoderValue;
  unsigned long initialTime = millis();
  
  delay(1000); // wait for 1 second
  
  int finalEncoderValue = encoderValue;
  unsigned long finalTime = millis();
  
  int deltaEncoderValue = finalEncoderValue - initialEncoderValue;
  unsigned long deltaTime = finalTime - initialTime;
  
  int encoderCPR = 600; // replace this with your encoder's CPR
  
  float velocity = (deltaEncoderValue / (float)deltaTime) * (2*PI / encoderCPR) * 1000; // velocity in rad/sec
  
  Serial.print("Velocity: ");
  Serial.print(velocity);
  Serial.println(" rad/sec");
  Serial.print(millis());
  
  Serial.print("Final Value: ");
  Serial.println(finalEncoderValue);
}

void updateEncoder() {
  int MSB = digitalRead(pin2); // MSB = most significant bit
  int LSB = digitalRead(pin3); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB; // converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; // adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue--;

  lastEncoded = encoded; // store this value for next time
}
