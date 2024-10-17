#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficient to convert duration to distance

#define _EMA_ALPHA 0.7  // EMA weight of new sample (range: 0 to 1)
                        // Adjust alpha to filter noise and maintain responsiveness

// Target Distance for servo control
#define _TARGET_LOW  180.0 // 18 cm (180 mm)
#define _TARGET_HIGH 360.0 // 36 cm (360 mm)

// duty duration for myservo.writeMicroseconds()
// Adjust these values based on your specific servo motor.
#define _DUTY_MIN 500   // 서보의 최소 신호 (0도)
#define _DUTY_MAX 2500  // 서보의 최대 신호 (180도)

// global variables
float dist_ema, dist_prev = _DIST_MAX; // unit: mm
unsigned long last_sampling_time;      // unit: ms

Servo myservo;

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);    // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);     // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_MIN); // Start at 0 degrees

  // initialize USS related variables
  dist_prev = _DIST_MIN; // raw distance output from USS (unit: mm)

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float dist_raw;

  if (millis() < (last_sampling_time + INTERVAL))
    return;

  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);
  
  // Apply range filter: ignore distances outside of 18~36cm
  if (dist_raw < _DIST_MIN || dist_raw > _DIST_MAX || dist_raw == 0.0) {
    dist_raw = dist_prev;       // Keep previous distance if out of range or invalid
    digitalWrite(PIN_LED, HIGH); // Turn on LED to indicate range issue
  } else {
    digitalWrite(PIN_LED, LOW);  // Turn off LED if valid distance is in range
    dist_prev = dist_raw;
  }

  // Apply EMA filter
  dist_ema = (_EMA_ALPHA * dist_raw) + ((1 - _EMA_ALPHA) * dist_ema);

  // adjust servo position based on the filtered distance
  // add your code here!
  int servo_pos = map(dist_ema, _TARGET_LOW, _TARGET_HIGH, 0, 180); // Map distance to servo angle
  servo_pos = constrain(servo_pos, 0, 180);                         // Constrain angle between 0 and 180 degrees
  int servo_duty = map(servo_pos, 0, 180, _DUTY_MIN, _DUTY_MAX);    // Convert angle to duty cycle

  myservo.writeMicroseconds(servo_duty);

  // Output distance and servo position to serial for plotter
  Serial.print("Min:");    Serial.print(_DIST_MIN);
  Serial.print(",dist:");  Serial.print(dist_raw);
  Serial.print(",ema:");   Serial.print(dist_ema);
  Serial.print(",Servo:"); Serial.print(servo_pos);
  Serial.print(",Max:");   Serial.print(_DIST_MAX);
  Serial.println("");

  // Update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
} 
