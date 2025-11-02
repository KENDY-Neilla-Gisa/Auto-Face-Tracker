// Face Tracker - Stepper Motor Controller
// For 28BYJ-48 stepper motor with ULN2003 driver

// Motor connections
const int IN1 = 8;
const int IN2 = 9;
const int IN3 = 10;
const int IN4 = 11;

// Motor control variables

// Variables for serial communication
String inputString = "";
bool stringComplete = false;

// Motor control variables
int currentStep = 0;
int targetPos = 0;
int currentPos = 0;
bool isMoving = false;
unsigned long lastStepTime = 0;
int stepDelay = 5;  // Initial delay between steps (ms)
int minStepDelay = 2;  // Minimum delay for max speed
int maxStepDelay = 10; // Maximum delay for slow speed
int acceleration = 1;  // Steps to accelerate/decelerate

void setup() {
  // Initialize serial communication
  Serial.begin(115200);  // Increased baud rate for faster communication
  inputString.reserve(32);
  
  // Set motor control pins as outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Initialize motor in the first position
  setMotor(stepSequence[0]);
  
  Serial.println("Face Tracker Ready!");
  Serial.println("Send commands: L# (left), R# (right), S (stop)");
}

void loop() {
  // Process any incoming serial data
  if (stringComplete) {
    handleCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  // Smooth movement with acceleration/deceleration
  if (isMoving && (micros() - lastStepTime) >= (stepDelay * 1000UL)) {
    if (currentPos < targetPos) {
      stepRight();
      currentPos++;
    } else if (currentPos > targetPos) {
      stepLeft();
      currentPos--;
    } else {
      isMoving = false;
      // Reset step delay when stopping
      stepDelay = maxStepDelay;
    }
    lastStepTime = micros();
    
    // Adjust speed based on distance to target (smooth acceleration/deceleration)
    int stepsToGo = abs(targetPos - currentPos);
    if (stepsToGo < 10) {
      // Slow down as we approach target
      stepDelay = map(stepsToGo, 0, 10, maxStepDelay, minStepDelay);
    } else {
      // Maintain a good speed for most of the movement
      stepDelay = minStepDelay;
    }
  }
}

// SerialEvent occurs whenever new data comes in the hardware serial RX
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0) {
        stringComplete = true;
      }
    } else if (isAlphaNumeric(inChar) || inChar == '-') {
      inputString += inChar;
    }
  }
}

void handleCommand(String command) {
  if (command.length() == 0) return;
  
  char cmd = command[0];
  int value = 0;
  
  // Parse the numeric value if present
  if (command.length() > 1) {
    value = command.substring(1).toInt();
    value = constrain(value, 1, 1000);  // Limit the maximum steps
  }
  
  // Execute the command
  switch (cmd) {
    case 'L':  // Move left by value steps
      targetPos = currentPos - value;
      isMoving = true;
      Serial.print("Moving left: ");
      Serial.println(value);
      break;
      
    case 'R':  // Move right by value steps
      targetPos = currentPos + value;
      isMoving = true;
      Serial.print("Moving right: ");
      Serial.println(value);
      break;
      
    case 'P':  // Move to absolute position
      targetPos = value;
      isMoving = true;
      Serial.print("Moving to position: ");
      Serial.println(value);
      break;
      
    case 'S':  // Stop
      isMoving = false;
      targetPos = currentPos;
      Serial.println("Stopped");
      break;
      
    default:
      Serial.print("Unknown command: ");
      Serial.println(cmd);
      break;
  }
}

void stepLeft() {
  currentStep = (currentStep - 1 + 8) % 8;
  setMotor(stepSequence[currentStep]);
  };
  
  static int step = 0;
  
  digitalWrite(IN1, steps[step][0]);
  digitalWrite(IN2, steps[step][1]);
  digitalWrite(IN3, steps[step][2]);
  digitalWrite(IN4, steps[step][3]);
  
  step++;
  if (step >= 8) step = 0;
}

// Rotate one step to the right
void stepRight() {
  // 8-step sequence for smoother operation (reversed order of stepLeft)
  const int steps[8][4] = {
    {0, 0, 0, 1},
    {0, 0, 1, 1},
    {0, 0, 1, 0},
    {0, 1, 1, 0},
    {0, 1, 0, 0},
    {1, 1, 0, 0},
    {1, 0, 0, 0},
    {1, 0, 0, 1}
  };
  
  static int step = 0;
  
  digitalWrite(IN1, steps[step][0]);
  digitalWrite(IN2, steps[step][1]);
  digitalWrite(IN3, steps[step][2]);
  digitalWrite(IN4, steps[step][3]);
  
  step++;
  if (step >= 8) step = 0;
}
