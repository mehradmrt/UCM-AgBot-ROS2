#define CPR 600 

int pinA = 2;
int pinB = 3; 
volatile int count = 0;
int lastCount = 0;
unsigned long lastTime;
const unsigned long sampleInterval = 50; //  milisecond sampling interval

void setup() {
  Serial.begin(9600);
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinA), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), ISR_B, CHANGE);
  lastTime = millis();
}

void loop() {
  // Serial.println(millis());
  if(millis() - lastTime >= sampleInterval) {
    float radPerSecond = (float)(count - lastCount)*(2*PI) / (4*(millis()-lastTime)*0.001*CPR);
    // Serial.print("rad/s: ");
    Serial.println(radPerSecond, 4); // 4 decimal places
    // Serial.println((float)(count - lastCount)/4);
    // Serial.println((millis()-lastTime)*0.001);

    lastCount = count;
    lastTime += sampleInterval; 
  }
}

void ISR_A() {
  if (digitalRead(pinA) != digitalRead(pinB)) {
    count--;
  } else {
    count++;
  }
}

void ISR_B() {
  if (digitalRead(pinB) != digitalRead(pinA)) {
    count++;
  } else {
    count--;
  }
}