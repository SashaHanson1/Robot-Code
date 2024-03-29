

const int trigPin = 39;                 // Trig pin
const int echoPin = 40;                 // Echo pin

float duration;                         // holds the length of the sound wave
float distance;                         // holds the distance to an object
const float calibration = 0;                   // calibration distance to ensure edge of robot is 0 distance

unsigned long previousMicros;           // last microsecond count
unsigned long currentMicros;            // current microsecond count

void setup() {
  pinMode(trigPin, OUTPUT);             // set Trig pin to output (emits ultrasonic waves)
	pinMode(echoPin, INPUT);              // set Echo pin to input (receives reflection waves)
	Serial.begin(9600);                   // start serial communication
} 

void loop() {  
  currentMicros = micros();                                                   // get current time in microseconds
  if ((currentMicros - previousMicros) >= 1000) {                             // enter when 1 ms has elapsed
    previousMicros = currentMicros;                                          // record current time in microseconds

    digitalWrite(trigPin, LOW);           // set Trig low
    delayMicroseconds(2);                 // ****** change this to a timer function not delay --- this delay is only meant 
    digitalWrite(trigPin, HIGH);          // set the trig to high for 10s which will send out an 8 cycle sonic burst
    delayMicroseconds(10);                // ****** change this to a timer function not delay
    digitalWrite(trigPin, LOW);           // set Trig low

    duration = pulseIn(echoPin, HIGH);    // set the Echo pin to high for the duration that the Trig pin was outputting the signal and store that length of time
    distance = ((duration*.0343)/2) - calibration;        // calculate distance to object using (time * speed of sound)/2 (divide by 2 because we only want the distance to object not to and from)
    
    // print distance
    Serial.print("Distance: ");  
    Serial.println(distance);  
   }
}
