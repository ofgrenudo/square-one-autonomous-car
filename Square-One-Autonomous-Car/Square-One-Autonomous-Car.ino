
// LED's
const int onboardLED = 13;

// Serial Communication
int Command = 0;

void setup() {
  Serial.begin(9600); // Start Serial Communications at 9600bps

  pinMode(onboardLED, OUTPUT);

  Serial.println("Initialization Complete!");
  Serial.println("Starting Now.....");  
  timed_delay_start();
}

void loop() {
  capture_task();
}

void timed_delay_start(){
  digitalWrite(onboardLED, HIGH);
  delay(500);
  digitalWrite(onboardLED, LOW);
  delay(500);

  digitalWrite(onboardLED, HIGH);
  delay(500);
  digitalWrite(onboardLED, LOW);
  delay(500);

  digitalWrite(onboardLED, HIGH);
  delay(500);
  digitalWrite(onboardLED, LOW);
  delay(500);

  digitalWrite(onboardLED, HIGH);
  delay(500);
  digitalWrite(onboardLED, LOW);
  delay(500);

  digitalWrite(onboardLED, HIGH);
  delay(500);
  digitalWrite(onboardLED, LOW);
  delay(500);
}

void capture_task() {
  if (Serial.available()>0) {

    if(Serial.parseInt() != 0){
      int x = Serial.parseInt();
      Serial.println(x);
      
      digitalWrite(onboardLED, HIGH);
      delay(500);
      digitalWrite(onboardLED, LOW);
      delay(100);      
    }
  }
}
