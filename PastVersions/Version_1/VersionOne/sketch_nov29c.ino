#define trigPin A4
#define echoPin A5
#define RedLED 9
#define buzzer 10

int sound = 500;


void setup() {
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(RedLED, OUTPUT);
  pinMode(buzzer, OUTPUT);
}

void loop() {
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/5) / 29.1;


  if (distance < 20) {
   digitalWrite(RedLED, HIGH);
   sound = 1000;
}
 else {
   digitalWrite(RedLED,LOW);
 }

 if (distance > 20 || distance <= 0){
      Serial.print(distance);
   Serial.println(" cm");
   noTone(buzzer);
 }
 else {
   Serial.print(distance);
   Serial.println(" cm");
   tone(buzzer, sound);

 }
delay(50);
}