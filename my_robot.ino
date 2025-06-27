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

  int kp = 50, kd = 500, PID_Value, P, D;
  float error; // Declare error variable
  static float previous_error = 0.0; // Made static to persist across calls
  int base_speed = 200, left_motor_speed, right_motor_speed, turn_speed = 100;
  static char t = 's'; // Made static to remember last turn direction, initialized to 'straight'

  // The while(1) loop is removed. This function will be called repeatedly by loop() when button is pressed.
  Sensor_reading();

  // If no sensors detect the line, avg might be calculated with total = 0, leading to issues.
  // Ensure total is not zero before calculating avg, or handle the case where total is 0.
  // Sensor_reading() already has 'if (total) avg = sensor_position / total;'
  // If total is 0, avg will retain its previous value. This might be problematic.
  // Let's ensure avg is well-defined. If total is 0, perhaps error should be max or based on 't'.

  if (total == 0) { // No line detected
    digitalWrite(led, HIGH); // Turn on LED to indicate line lost
    // Robot continues with last known turn direction 't' or a search pattern
    if (t == 'r') {
      motor(turn_speed, -turn_speed); // Turn right
    } else if (t == 'l') {
      motor(-turn_speed, turn_speed); // Turn left
    } else {
      // Default behavior if 't' is 's' (straight) or undefined: maybe stop or search.
      // For now, let's make it turn one way, e.g. right.
      motor(turn_speed, -turn_speed);
    }
    // No PID calculation if line is lost, just try to find it.
    // previous_error should probably be reset or not updated if the line is lost.
    // previous_error = 0; // Optional: Reset error when line is lost
    return; // Exit PID_LINE_FOLLOW, motor commands for search are set.
  }
  
  // If we have a line (total > 0), proceed with PID.
  // avg is calculated in Sensor_reading() only if total is non-zero.
  error = set_point - avg;
  D = kd * (error - previous_error);
    P = error * kp;
    PID_Value = P + D;
    previous_error = error;

    //adjusting motor speed to keep the robot in line
    right_motor_speed = base_speed - PID_Value;  //right motor speed
    left_motor_speed = base_speed + PID_Value;  //left motor speed
    motor(left_motor_speed, right_motor_speed);  // Apply PID adjusted speed

    // Update last turn direction 't' based on edge sensors
    // This logic should only set 't' if the line is clearly to one side.
    // It's also used for the "line lost" scenario.
    if (s[0] == 1 && s[4] == 0) t = 'r'; // Line far to the right (robot needs to turn hard right)
    else if (s[4] == 1 && s[0] == 0) t = 'l'; // Line far to the left (robot needs to turn hard left)
    // else if (total > 0 && total < 5) t = 's'; // If some sensors see the line, but not all, assume it's somewhat straight or correcting.

    // Handling for intersections (all sensors black)
    // This 'else if (total == 5)' was inside the while(1) loop.
    // Now, it's part of a single pass of PID_LINE_FOLLOW().
    if (total == 5) { 
      // All sensors see black - could be an intersection or end of line marker
      motor(0, 0); // Stop at intersection
      // To proceed past an intersection, more logic would be needed (e.g., go straight for a bit, then re-evaluate)
      // For now, it just stops.
      // We might want to set a flag or special state here.
      // If it stops, subsequent calls to PID_LINE_FOLLOW (if button is still pressed)
      // will re-evaluate. If it's still total == 5, it will stop again.
      // This is different from 'while (total == 5) Sensor_reading();' which would get stuck.
      // The original code 'else if (total == 0) t = 's';' after this block was confusing.
    }
    
    // The LED that was turned on when total == 0 should be turned off if line is found again.
    // Or, use LED to indicate line following is active.
    digitalWrite(led, LOW); // Assuming LED HIGH meant line lost, turn it off if line is found/processed.
                            // This might conflict with LED blinking in loop().
                            // Let's reconsider LED usage. The loop() blinks then calls this.
                            // Maybe LED in PID_LINE_FOLLOW is for error states only.
                            // For now, if total !=0, we are "on line", turn off "line lost" indicator.

} // End of PID_LINE_FOLLOW function (removed while(1))

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
