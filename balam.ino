#include <Servo.h>
#include <Arduino_FreeRTOS.h>

Servo barrier; // Servo motor for the barrier

// Ultrasonic sensor pins for entry detection
const int trigPin = 4;
const int echoPin = 2;

// Ultrasonic sensor pins for exit detection
const int trigPinExit = 6;
const int echoPinExit = 7;

// Servo motor control pin
const int servoPin = 10;

// Distance threshold for detecting a car (in cm)
const int carDistanceThreshold = 15;

// Shared variables for communication between tasks
volatile bool carDetected = false;
volatile bool carExitDetected = false;

void setup() {
  Serial.begin(9600);

  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(trigPinExit, OUTPUT);
  pinMode(echoPinExit, INPUT);

  // Attach the servo motor
  barrier.attach(servoPin);

  // Start with the barrier closed
  barrier.write(0); // 0 degrees (barrier down)

  // Create tasks
  xTaskCreate(taskUltrasonic, "UltrasonicTask", 128, NULL, 2, NULL);
  xTaskCreate(taskServoControl, "ServoControlTask", 128, NULL, 1, NULL);
}

void loop() {
  // FreeRTOS loop - nothing to do here
}

// Task to read both ultrasonic sensors and detect cars
void taskUltrasonic(void* pvParameters) {
  (void) pvParameters;

  while (1) {
    // Detect entry
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long durationEntry = pulseIn(echoPin, HIGH);
    int distanceEntry = durationEntry * 0.034 / 2;

    if (distanceEntry > 0 && distanceEntry <= carDistanceThreshold) {
      carDetected = true;
    } else {
      carDetected = false;
    }

    // Detect exit
    digitalWrite(trigPinExit, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinExit, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinExit, LOW);
    long durationExit = pulseIn(echoPinExit, HIGH);
    int distanceExit = durationExit * 0.034 / 2;

    if (distanceExit > 0 && distanceExit <= carDistanceThreshold) {
      carExitDetected = true;
    } else {
      carExitDetected = false;
    }

    // Print distances for debugging
    Serial.print("Entry Distance: ");


    Serial.print("Exit Distance: ");
    Serial.print(distanceEntry);
    Serial.println(" cm");

    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 100ms
  }
}

// Task to control the servo motor based on car detection
void taskServoControl(void* pvParameters) {
  (void) pvParameters;

  while (1) {
    if (carDetected || carExitDetected) {
      // Open the barrier
      Serial.println("Car detected! Opening barrier...");
      barrier.write(90); // 90 degrees (barrier up)

      // Wait for 3 seconds
      vTaskDelay(3000 / portTICK_PERIOD_MS);

      // Close the barrier
      Serial.println("Closing barrier...");
      barrier.write(0); // 0 degrees (barrier down)

      // Reset detection flags
      carDetected = false;
      carExitDetected = false;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 100ms
  }
}
