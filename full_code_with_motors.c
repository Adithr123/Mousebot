//Owned by Maker Nexus 2022

#include <AFMotor.h>  // tells the IDE we are using the motor shield library

AF_DCMotor right_wheel(1, MOTOR12_64KHZ); // create motor #1, 64KHz pwm
AF_DCMotor left_wheel(2, MOTOR12_64KHZ); // create motor #2, 64KHz pwm

#define minimum_motor_speed 100
#define debounce_threshold_ms 500

#define line_following_speed 150

// reflectance pins
#define reflector_left_pin A3
#define reflector_right_pin A2

// control modes
#define JOYSTICK_MODE 0
#define ULTRASOUND_MODE 1
#define LINE_MODE 2

// ultrasonic detector pins
#define echo_pin 5
#define trigger_pin 6

// whisker switch pins
#define left_whisker_pin A0
#define right_whisker_pin 2

// joystick pins & behavior                                                          
#define joystick_x_pin A4
#define joystick_y_pin A5
#define joystick_switch_pin 9
#define joystick_dead_zone 20

// joystick variables
int joystick_x_position = 0;
int joystick_y_position = 0;
int prior_joystick_switch_position = 1;
long button_last_pressed = millis();  // global variable to track when button last pressed

int mode = JOYSTICK_MODE;

void RightMotorControl(int speed){
  right_wheel.setSpeed(abs(speed));
  if(speed > 0)
    right_wheel.run(FORWARD);
  else if(speed < 0)
    right_wheel.run(BACKWARD);
  else 
    right_wheel.run(RELEASE);
}


void LeftMotorControl(int speed){
  left_wheel.setSpeed(abs(speed));
  if(speed > 0)
    left_wheel.run(FORWARD);
  else if(speed < 0)
    left_wheel.run(BACKWARD);
  else 
    left_wheel.run(RELEASE);
}


void WheelSpeedsLeftRight(int left_speed, int right_speed) {
  Serial.print("LEFT speed ");
  Serial.print(left_speed);
  Serial.print(" | RIGHT speed ");
  Serial.println(right_speed);
  RightMotorControl(right_speed);
  LeftMotorControl(left_speed);
}


void TurnRightDuration(float seconds, int speed) {
  Serial.print("RIGHT turn at speed ");
  Serial.print(speed);
  Serial.print(" for ");
  Serial.print(seconds);
  Serial.println(" seconds");
  RightMotorControl(0);
  LeftMotorControl(speed);
  delay(seconds*1000);
}


void TurnLeftDuration(float seconds, int speed) {
  Serial.print("LEFT turn at speed ");
  Serial.print(speed);
  Serial.print(" for ");
  Serial.print(seconds);
  Serial.println(" seconds");
  RightMotorControl(speed);
  LeftMotorControl(0);
  delay(seconds*1000);
}


void StraightDuration(float seconds, int speed) {
  if(speed > 0)
    Serial.print("FORWARD at speed ");
  else if(speed < 0)
    Serial.print("REVERSE at speed ");
  else
    Serial.print("STOPPED at speed ");
  Serial.print(abs(speed));
  Serial.print(" for ");
  Serial.print(seconds);
  Serial.println(" seconds");
  RightMotorControl(speed);
  LeftMotorControl(speed);
  delay(seconds*1000);
}


void Straight(int speed) {
  if(speed > 0)
    Serial.print("FORWARD at speed ");
  else if(speed < 0)
    Serial.print("REVERSE at speed ");
  else
    Serial.print("STOPPED at speed ");
  Serial.print(abs(speed));
  Serial.println(" indefinitely");
  RightMotorControl(speed);
  LeftMotorControl(speed);
}


void TurnLeft(int speed) {
  Serial.print("LEFT turn at speed ");
  Serial.println(speed);
  RightMotorControl(speed);
  LeftMotorControl(0);
}


void TurnRight(int speed) {
  Serial.print("RIGHT turn at speed ");
  Serial.println(speed);
  RightMotorControl(0);
  LeftMotorControl(speed);
}


void Stop() {
  Serial.println("STOP");
  RightMotorControl(0);
  LeftMotorControl(0);
}


bool IsWhite(int pin_number){
  int reflectance_value = analogRead(pin_number);
  if(reflectance_value < 600)
    return true;
  return false;
  // could be replaced by single line
  // return analogRead(pin_number) < 600;
}

bool IsBlack(int pin_number){
  return !IsWhite(pin_number);
}


bool InAir(int pin_number){
  return analogRead(pin_number) > 900;
}


bool JoystickSwitchFirstPressed(){
  int current_joystick_switch_position = digitalRead(joystick_switch_pin);
  bool pushed = false;

  long now = millis();  // the number of milliseconds since Arduino powered on
  if (prior_joystick_switch_position == 1 &&
      current_joystick_switch_position == 0 &&
      now - button_last_pressed > debounce_threshold_ms){
    pushed = true;
    button_last_pressed = now;
    Stop();
  }

  prior_joystick_switch_position = current_joystick_switch_position;
  return pushed; 
}


void PrintMode(){
  Serial.print("Mode (");
  Serial.print(millis());
  Serial.print("): ");
  if(mode == 0)
    Serial.println("joystick");
  else if(mode == 1)
    Serial.println("autonomous");
  else if(mode == 2)
    Serial.println("line following");
  else
    Serial.println("ERROR");
}


int UltrasoundDistanceCentimeters(){
  int distance; // variable for the distance measurement
  long duration; // variable for the duration of sound wave travel
  digitalWrite(trigger_pin, LOW);
  delayMicroseconds(2);
  // Sets the trigger_pin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger_pin, LOW);
  // Reads the echo_pin, returns the sound wave travel time in microseconds
  duration = pulseIn(echo_pin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  return distance;
}


void setup() {
  Serial.begin(9600); 
  pinMode(joystick_switch_pin, INPUT_PULLUP);

  pinMode(joystick_x_pin, INPUT);
  pinMode(joystick_y_pin, INPUT);

  pinMode(trigger_pin, OUTPUT);
  pinMode(echo_pin, INPUT);

  pinMode(left_whisker_pin, INPUT_PULLUP);
  pinMode(right_whisker_pin, INPUT_PULLUP);

  PrintMode();
}


void loop() {
  if (JoystickSwitchFirstPressed()){
    mode = mode + 1;
    if (mode > 2)
      mode = 0;
    PrintMode();
  }
  if (mode == JOYSTICK_MODE) {
    // SPEED
    joystick_x_position = analogRead(joystick_x_pin);  // forward / backward
    int joystick_speed = abs(joystick_x_position - 512);
    if (joystick_speed < joystick_dead_zone)
      joystick_speed = 0;
    else
      joystick_speed = map(joystick_speed, joystick_dead_zone, 512, minimum_motor_speed, 255);
    if(joystick_x_position - 512 < 0)
      joystick_speed = joystick_speed * -1;

    // TURNING
    joystick_y_position = analogRead(joystick_y_pin);  // left / right
    int joystick_direction = abs(joystick_y_position - 512);  // re-centered -511 .. 511
    if (joystick_direction < joystick_dead_zone)  // ignores values less than 20
      joystick_direction = 0;
    else
      joystick_direction = map(joystick_direction, joystick_dead_zone, 511, 0, 100);  // remaps result to a percent 0..100
    joystick_direction = 100 - joystick_direction; // we want a value of 100% at the center & 0 at extremes
    float direction_multiplier = joystick_direction / 100.0;

    bool veer_left = (joystick_y_position - 512 < 0);

    int right_wheel_speed = joystick_speed;
    int left_wheel_speed = joystick_speed;

    if (veer_left)
      left_wheel_speed = left_wheel_speed * direction_multiplier;
    else
      right_wheel_speed = right_wheel_speed * direction_multiplier;

    WheelSpeedsLeftRight(left_wheel_speed, right_wheel_speed);

  } else if (mode == ULTRASOUND_MODE) {
    bool right_whisker_free = digitalRead(right_whisker_pin);
    bool left_whisker_free = digitalRead(left_whisker_pin);
    if(!right_whisker_free)
      Serial.println("Right whisker hit an obstacle!");
    if(!left_whisker_free)
      Serial.println("Left whisker hit an obstacle!");
    int distance_centimeters = UltrasoundDistanceCentimeters();
    if (distance_centimeters >= 5 && right_whisker_free && left_whisker_free){
      Straight(200);
    } else if (distance_centimeters < 5) {
      StraightDuration(0.5, -200);
      TurnLeftDuration(0.5, 200);
    } else if (!right_whisker_free) {
      StraightDuration(0.5, -200);
      TurnLeftDuration(0.5, 200);
    } else if (!left_whisker_free) {
      StraightDuration(0.5, -200);
      TurnRightDuration(0.5, 200);
    } else {
      Stop();
    }

  } else {  // mode is LINE_MODE
    int distance_centimeters = UltrasoundDistanceCentimeters();
    bool obstacle_in_front = distance_centimeters < 10;
    bool in_air = InAir(reflector_left_pin) && InAir(reflector_right_pin);

    if(obstacle_in_front){
      Serial.print("Obstacle ");
      Serial.print(distance_centimeters);
      Serial.print(" cm away | ");
      Stop();
    }
    else if(in_air){
      Serial.print("MouseBot might be up in the air | ");
      Stop();
    }
    else if(IsWhite(reflector_left_pin) && IsWhite(reflector_right_pin)){  // centered
      Serial.print("Centered | ");      
      Straight(line_following_speed);
    } else if (IsBlack(reflector_left_pin) && IsWhite(reflector_right_pin)) {  // car is too far to the right; go left
      Serial.print("Line detected on left | ");      
      // TurnLeft(line_following_speed);  // Need to pivot a bit more than just turn
      WheelSpeedsLeftRight(-line_following_speed, line_following_speed);
    } else if (IsWhite(reflector_left_pin) && IsBlack(reflector_right_pin)) {  // car is too far to the left; go right
      Serial.print("Line detected on right | ");      
      //TurnRight(line_following_speed);  // Need to pivot a bit more than just turn
      WheelSpeedsLeftRight(line_following_speed, -line_following_speed);
    } else {  // uh-oh!
      Serial.print("Line detected by both | ");      
      Stop();
    }
  }

}