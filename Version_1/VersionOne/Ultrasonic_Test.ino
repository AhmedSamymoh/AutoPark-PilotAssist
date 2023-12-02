// Define the pins for the ultrasonic sensor
const int trigPin = 5;  // Trig pin of the ultrasonic sensor
const int echoPin = 4; // Echo pin of the ultrasonic sensor

// Define the pin for the buzzer
const int buzzerPin = 10;


void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
}

void loop() {
  // Trigger the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the time taken for the ultrasonic pulse to return
  long duration = pulseIn(echoPin, HIGH);

  // Calculate distance in centimeters
  int distance = duration * 0.034 / 2;




  // Check the distance and control the buzzer
  if (distance < 10) {
    // If the distance is less than 20 cm, beep the buzzer
    tone(buzzerPin, 1000); // Beep at 1000 Hz
  } else {
    // If the distance is 20 cm or more, turn off the buzzer
    noTone(buzzerPin);
  }
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}
