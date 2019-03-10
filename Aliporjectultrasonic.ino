

const int TrigPin1 = 4;
const int EchoPin1 = 3;

const int TrigPin2 = 5;
const int EchoPin2 = 6;

const int TrigPin3 = 9;
const int EchoPin3 = 8;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(TrigPin1, OUTPUT);
  pinMode(EchoPin1, INPUT);

  pinMode(TrigPin2, OUTPUT); 
  pinMode(EchoPin2, INPUT);

  pinMode(TrigPin3, OUTPUT);
  pinMode(EchoPin3, INPUT);

  pinMode(7, OUTPUT);
  
}

void loop() {
digitalWrite(TrigPin1, LOW);
delayMicroseconds(2);
digitalWrite(TrigPin1, HIGH);
delayMicroseconds(10);
digitalWrite(TrigPin1, LOW);
float distance1 = pulseIn(EchoPin1, HIGH)*(0.034/2);

digitalWrite(TrigPin2, LOW);
delayMicroseconds(2);
digitalWrite(TrigPin2, HIGH);
delayMicroseconds(10);
digitalWrite(TrigPin2, LOW);
float distance2 = pulseIn(EchoPin2, HIGH)*(0.034/2);

digitalWrite(TrigPin3, LOW);
delayMicroseconds(2);
digitalWrite(TrigPin3, HIGH);
delayMicroseconds(10);
digitalWrite(TrigPin3, LOW);
float distance3 = pulseIn(EchoPin3, HIGH)*(0.034/2);

Serial.print("distance1 = ");
Serial.print(distance1);
Serial.print("  distance2 = ");
Serial.print(distance2);
Serial.print("  distance3 = ");
Serial.println(distance3);
delay(100);
if(distance1 < 11 || distance2 < 11 || distance3 < 11 ) {
    digitalWrite(7, HIGH);
}
else {
    digitalWrite(7, LOW);
}
}
