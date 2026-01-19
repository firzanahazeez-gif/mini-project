// Mini Washer Machine with HC-05 Bluetooth
// Sends status updates to phone via Bluetooth Serial Terminal

#include <SoftwareSerial.h>

// Bluetooth Serial (RX=Pin 10, TX=Pin 11)
SoftwareSerial BTSerial(10, 11); // RX, TX

// Pin Definitions
const int BUTTON_PIN = 2;      // Push button input
const int LED_PIN = 13;        // LED indicator
const int MOTOR_ENA = 9;       // Motor speed control (PWM)
const int MOTOR_IN1 = 7;       // Motor direction 1
const int MOTOR_IN2 = 6;       // Motor direction 2

// Wash cycle parameters
const int MOTOR_SPEED = 255;   // Speed (0-255), MAX for 5V
const int WASH_TIME = 10000;   // Washing time (10 seconds)
const int SPIN_TIME = 5000;    // Spinning time (5 seconds)

// State variables
bool isWashing = false;
bool buttonPressed = false;
unsigned long startTime = 0;
unsigned long lastUpdate = 0;

void setup() {
  // Initialize Hardware Serial (for debugging - optional)
  Serial.begin(9600);
  Serial.println("Mini Washer Machine with Bluetooth");
  
  // Initialize Bluetooth Serial (HC-05 default: 9600 baud)
  BTSerial.begin(9600);
  delay(100);
  
  // Send startup message to phone
  BTSerial.println("============================");
  BTSerial.println("Mini Washer Machine Ready!");
  BTSerial.println("Press button to start wash");
  BTSerial.println("============================");
  
  // Initialize pins
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  
  // Ensure motor is stopped
  stopMotor();
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  // Read button state
  bool currentButtonState = (digitalRead(BUTTON_PIN) == LOW);
  
  // Detect button press
  if (currentButtonState && !buttonPressed && !isWashing) {
    startWashCycle();
  }
  
  buttonPressed = currentButtonState;
  
  // Handle wash cycle
  if (isWashing) {
    unsigned long elapsedTime = millis() - startTime;
    unsigned long totalTime = WASH_TIME + SPIN_TIME;
    unsigned long remainingTime = totalTime - elapsedTime;
    
    // Send status update every 2 seconds
    if (millis() - lastUpdate >= 2000) {
      lastUpdate = millis();
      sendStatusUpdate(elapsedTime, remainingTime);
    }
    
    if (elapsedTime < WASH_TIME) {
      // Washing phase
      rotateForward();
    } else if (elapsedTime < totalTime) {
      // Spinning phase
      rotateBackward();
    } else {
      // Cycle complete
      stopWashCycle();
    }
  }
}

// Start wash cycle
void startWashCycle() {
  isWashing = true;
  startTime = millis();
  lastUpdate = millis();
  digitalWrite(LED_PIN, HIGH);
  
  // Send to phone
  BTSerial.println("\n============================");
  BTSerial.println("WASH CYCLE STARTED!");
  BTSerial.println("============================");
  
  // Debug
  Serial.println("Wash cycle started");
}

// Stop wash cycle
void stopWashCycle() {
  isWashing = false;
  stopMotor();
  digitalWrite(LED_PIN, LOW);
  
  // Send completion message to phone
  BTSerial.println("\n****************************");
  BTSerial.println("YOUR WASHING IS DONE!");
  BTSerial.println("****************************");
  BTSerial.println("Press button to start again\n");
  
  // Debug
  Serial.println("Wash cycle complete");
}

// Send status update to phone
void sendStatusUpdate(unsigned long elapsed, unsigned long remaining) {
  int remainingSec = remaining / 1000;
  int minutes = remainingSec / 60;
  int seconds = remainingSec % 60;
  
  // Create status message
  BTSerial.print("Status: ");
  
  if (elapsed < WASH_TIME) {
    BTSerial.print("WASHING");
  } else {
    BTSerial.print("SPINNING");
  }
  
  BTSerial.print(" | Time left: ");
  if (minutes < 10) BTSerial.print("0");
  BTSerial.print(minutes);
  BTSerial.print(":");
  if (seconds < 10) BTSerial.print("0");
  BTSerial.println(seconds);
}

// Motor control functions
void rotateForward() {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_ENA, MOTOR_SPEED);
}

void rotateBackward() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  analogWrite(MOTOR_ENA, MOTOR_SPEED);
}

void stopMotor() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_ENA, 0);
}
}
