#define PARK_FRONT  A0
#define PARK_REAR   A1
#define PARK_STATUS 6
#define ACC         2

void setup() {
  pinMode(PARK_FRONT, OUTPUT);
  pinMode(PARK_REAR, OUTPUT);
  pinMode(PARK_STATUS, OUTPUT);
  pinMode(ACC, INPUT);
  
//  attachInterrupt(digitalPinToInterrupt(ACC), ACC_ISR, CHANGE);
  
//  Serial.begin(115200);
//  Serial.println("System Status: On");
}

void loop() {
  if(digitalRead(ACC)) {
    enableParkMode();
  } else {
    disableParkMode();
  }
  delay(200);
}

//void ACC_ISR() {
//  if(digitalRead(ACC)) {
//    enableParkMode();
//  } else {
//    disableParkMode();
//  }
//}

void enableParkMode() {
  digitalWrite(PARK_FRONT, LOW);
  digitalWrite(PARK_REAR, LOW);
  digitalWrite(PARK_STATUS, LOW);
//  Serial.println("Park Mode: Enabled");
}

void disableParkMode() {
  digitalWrite(PARK_FRONT, HIGH);
  digitalWrite(PARK_REAR, HIGH);
  digitalWrite(PARK_STATUS, HIGH);
//  Serial.println("Park Mode: Disabled");
}
