//motor pin defines
#define IN1 6
#define IN2 5
#define IN3 4
#define IN4 2
#define enA 9
#define enB 3

//declaration of necessary variable to line follow
int s[5], total, sensor_position; // Corrected size to 5 for 5 sensors
int threshold = 531; //set threshold as like i've shown in video
float avg;
int position[5] = { 1, 2, 3, 4, 5 }; // Corrected size to 5 for 5 sensors
int set_point = 3;

//push button and led
int button_pin = 12;
int led = 13;
// bool button_pin_state; // No longer needed
const byte analogPins[5] = {A4, A3, A2, A1, A0};

// New constants and variables for turn handling
const int TURN_DETECT_DELAY = 200;    // Milliseconds to wait after detecting a potential turn
bool expectingTurn = false;           // True if robot is in "go straight then re-evaluate" phase
unsigned long turnDetectStartTime = 0; // Timestamp when a potential turn was detected
char pendingTurnDirection = 's';      // 's' = straight, 'l' = left, 'r' = right

// Global state variables for toggle button logic
bool lineFollowingActive = false;     // Is the robot currently supposed to be line following?
int buttonState;                      // Current debounced state of the button
int lastSteadyButtonState = HIGH;     // Previous raw reading of the button, for debounce timing
unsigned long lastDebounceTime = 0;   // Used for button debouncing
unsigned long debounceDelay = 50;     // Debounce time in milliseconds

void setup() {
  //motor driver pins as output
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  //button pin as input
  pinMode(button_pin, INPUT_PULLUP);
  //led pin as output
  pinMode(led, OUTPUT);
  Serial.begin(9600);
  buttonState = digitalRead(button_pin); // Initialize buttonState
  lastSteadyButtonState = buttonState;   // Initialize with current button state
}

void loop() {
  display_value();

  int reading = digitalRead(button_pin); // Raw reading

  // If the raw reading has changed, reset the debouncing timer.
  // This tracks the time since the last *change* in raw signal.
  if (reading != lastSteadyButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // The reading has been stable for longer than the debounce delay.
    // If this stable reading is different from the current debounced buttonState,
    // it means a confirmed change has occurred.
    if (reading != buttonState) {
      buttonState = reading; // Update the debounced state

      // Toggle happens only on the falling edge (press: HIGH -> LOW)
      if (buttonState == LOW) {
        lineFollowingActive = !lineFollowingActive; // Toggle the mode

        if (lineFollowingActive) {
          // Just transitioned to active: blink LEDs as a start signal
          for (int i = 0; i < 3; ++i) {
            digitalWrite(led, HIGH);
            delay(100);
            digitalWrite(led, LOW);
            delay(100);
          }
        } else {
          // Just transitioned to inactive: ensure LED is off and motors stop
          digitalWrite(led, LOW);
          motor(0, 0);
        }
      }
    }
  }

  lastSteadyButtonState = reading; // Save the current raw reading for the next loop iteration, to detect changes.

  // Act based on the lineFollowingActive state
  if (lineFollowingActive) {
    PID_LINE_FOLLOW(); // This will be called continuously if active
  }
  // Motors are stopped and LED turned off when lineFollowingActive becomes false.
  // No need for an 'else' block here to actively stop them on every loop if already inactive.
}

void Sensor_reading() {
  sensor_position = 0;
  total = 0;
  for (byte i = 0; i < 5; i++) {  // Sensor data read from A0, A1, A2, A3, A4
    s[i] = analogRead(analogPins[i]);
    // If analogRead is LOW (sees black), set s[i] to 1. Otherwise, set to 0 (sees white).
    if (s[i] < threshold) s[i] = 1;  // 1 means black detected
    else s[i] = 0;                   // 0 means white detected
    sensor_position += s[i] * position[i];
    total += s[i];
  }
  if (total) avg = sensor_position / total;  //average value
}

void PID_LINE_FOLLOW() {

  int kp = 200, kd = 300, PID_Value, P, D;
  float error; // Declare error variable
  static float previous_error = 0.0; // Made static to persist across calls
  int base_speed = 240, left_motor_speed, right_motor_speed, turn_speed = 100;
  static char t = 's'; // Made static to remember last turn direction, initialized to 'straight'

  Sensor_reading(); // Initial sensor reading for the current cycle

  // Handle the "expectingTurn" state first
  if (expectingTurn) {
    if (millis() - turnDetectStartTime >= TURN_DETECT_DELAY) {
      // Delay has passed, re-evaluate sensors
      Sensor_reading(); // Get fresh sensor data

      bool forwardLineDetected = (s[1] == 1 || s[2] == 1 || s[3] == 1);
      // bool centerSensorOnLine = (s[2] == 1); // More specific check for straight

      if (forwardLineDetected) {
        // Prioritize forward: Line detected in front (especially s[1], s[2] or s[3])
        // Proceed with normal PID for this iteration (will effectively go straight or correct slightly)
        // No special motor command here, let PID take over.
      } else {
        // No forward line detected. Check if the pending turn is still indicated by sensors.
        if (pendingTurnDirection == 'l' && (s[4] == 1 || s[3] == 1)) { // Check s[4] or s[3] for left
          motor(-turn_speed, turn_speed); // Execute left turn
        } else if (pendingTurnDirection == 'r' && (s[0] == 1 || s[1] == 1)) { // Check s[0] or s[1] for right
          motor(turn_speed, -turn_speed); // Execute right turn
        } else {
          // No line detected at all or side line disappeared, fall through to line lost (total == 0) logic below
          // The 'total' will be 0 or low, so the existing line lost logic will trigger.
        }
      }
      expectingTurn = false; // Reset state
      pendingTurnDirection = 's'; // Reset pending turn
      // After this block, normal PID or line-lost logic will take over based on new Sensor_reading()
    } else {
      // Delay has not passed, continue straight
      motor(base_speed, base_speed); // Or a defined straight speed for this phase
      return; // Skip PID and other logic for this iteration
    }
  }

  // Standard Line Lost and Intersection Logic (executed if not in `expectingTurn` delay phase, or after it completes)
  if (total == 0) { // No line detected
    digitalWrite(led, HIGH); // Indicate line lost
    if (t == 'r') {
      motor(turn_speed, -turn_speed);
    } else if (t == 'l') {
      motor(-turn_speed, turn_speed);
    } else {
      motor(turn_speed, -turn_speed); // Default search: turn right
    }
    // previous_error = 0; // Optional: Reset error when line is lost
    return;
  }

  if (total == 5) { // All sensors see black - intersection
    motor(0, 0); // Stop at intersection
    // Potentially set expectingTurn = true, pendingTurnDirection = 's' (or based on rules),
    // and turnDetectStartTime = millis() to go straight over intersection for TURN_DETECT_DELAY.
    // For now, just stops as per original refined logic.
    // If we want to cross, we might need more states or rules.
    // Example: `expectingTurn = true; pendingTurnDirection = 's'; turnDetectStartTime = millis(); motor(base_speed, base_speed); return;`
    return; // Stop and wait for next cycle or manual restart for now.
  }

  // If not expecting a turn (or finished expecting one and line is found),
  // and not line-lost or at an intersection, then detect potential new turns or do PID.
  if (!expectingTurn) { // This check ensures we don't re-trigger turn detection immediately after finishing one.
    // Detect Potential Turns (if middle sensor is off the line, and an edge one is on)
    // s[2] == 0 (middle sensor NOT on line)
    // s[1] == 0 (inner right sensor NOT on line)
    // s[3] == 0 (inner left sensor NOT on line)
    if (s[0] == 1 && s[1] == 0 && s[2] == 0) { // Potential sharp right turn
      expectingTurn = true;
      turnDetectStartTime = millis();
      pendingTurnDirection = 'r';
      motor(base_speed, base_speed); // Start going straight
      return;
    } else if (s[4] == 1 && s[3] == 0 && s[2] == 0) { // Potential sharp left turn
      expectingTurn = true;
      turnDetectStartTime = millis();
      pendingTurnDirection = 'l';
      motor(base_speed, base_speed); // Start going straight
      return;
    }
  }

  // PID Calculation and Normal Line Following (if no special states/conditions above are met)
  // This part is reached if:
  // - not in `expectingTurn` delay
  // - line is detected (total > 0 and total < 5)
  // - no new sharp turn was just detected in this cycle

  error = set_point - avg; // avg should be valid here because total > 0
  P = error * kp;
  D = kd * (error - previous_error);
  PID_Value = P + D;
  previous_error = error;

  right_motor_speed = base_speed - PID_Value;
  left_motor_speed = base_speed + PID_Value;
  motor(left_motor_speed, right_motor_speed);

  // Update last turn direction 't' for line loss recovery
  // This logic might need refinement based on how well new turn logic works.
  // For example, if s[0] and s[4] are used by new turn logic, 't' might not get set for hard turns.
  // However, 't' is more for gradual loss or when line ends.
  if (s[0] == 1 && s[1] == 0 && s[2] == 0 && s[3] == 0 && s[4] == 0) t = 'r'; // Far right
  else if (s[4] == 1 && s[0] == 0 && s[1] == 0 && s[2] == 0 && s[3] == 0) t = 'l'; // Far left
  // Consider if 't' should be 's' if robot is generally on line, e.g. total >=1 && total <=3
  // else if (total > 0 && total < 5 ) t = 's'; // This might override 't' too eagerly.

  digitalWrite(led, LOW); // If line is being followed, turn off "line lost" indicator.

} // End of PID_LINE_FOLLOW function

void display_value() {  //display the analog value of sensor in serial monitor
  for (byte i = 0; i < 5; i++) {
    s[i] = analogRead(analogPins[i]);
    //Serial.print(String(s[i]) + " ");
  }
  //Serial.println();
  //delay(50);
}

//motor run function
// Parameter 'left_speed_cmd' controls motor connected to IN3, IN4, enB
// Parameter 'right_speed_cmd' controls motor connected to IN1, IN2, enA
void motor(int left_speed_cmd, int right_speed_cmd) {
  int actual_left_speed;
  int actual_right_speed;

  // Control Left Motor (IN3, IN4, enB)
  if (left_speed_cmd >= 0) { // Forward or stop
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    actual_left_speed = left_speed_cmd;
  } else { // Backward
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    actual_left_speed = -left_speed_cmd; // Make speed positive for analogWrite
  }

  // Control Right Motor (IN1, IN2, enA)
  if (right_speed_cmd >= 0) { // Forward or stop
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    actual_right_speed = right_speed_cmd;
  } else { // Backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    actual_right_speed = -right_speed_cmd; // Make speed positive for analogWrite
  }

  // Cap speeds to 0-255 range for analogWrite
  if (actual_left_speed > 255) actual_left_speed = 255;
  if (actual_right_speed > 255) actual_right_speed = 255;
  // analogWrite also handles values < 0 by treating them as 0, but explicit check is clearer.
  if (actual_left_speed < 0) actual_left_speed = 0; 
  if (actual_right_speed < 0) actual_right_speed = 0;


  analogWrite(enB, actual_left_speed);  // enB for Left Motor
  analogWrite(enA, actual_right_speed); // enA for Right Motor
  
  Serial.print("L:");
  Serial.print(actual_left_speed);
  Serial.print(" R:");
  Serial.println(actual_right_speed);
  // delay(50); // This delay might be too long for responsive PID control. Consider reducing or removing.
}
