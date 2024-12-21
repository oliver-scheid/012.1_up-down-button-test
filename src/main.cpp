#include <Arduino.h>
#include <math.h>

// defines
#define BTN_UP_PIN_ 2   // Pin 2 am ESP32, "[ABORT]" on cnc-shield -> used for digital output from button UP
#define BTN_DOWN_PIN_ 4 // Pin 4 am ESP32, "[HOLD]" on cnc-shield -> used for digital output form button DOWN
//
#define LED_BTN_UP_PIN_ 18   // Pin 18 am ESP32, "[SPN DIR]" on cnc-shield -> used for controlling LED -> LED on when button up is pressed
#define LED_BTN_DOWN_PIN_ 19 // Pin 19 am ESP32, "[SPN EN]" on cnc-shield -> used for controlling LED -> LED on when button down is pressed
//

//  Defines: "USE_WITH_CNCSHIELD" or "BARE_ESP32" -> use either of the two depending on your physical set up (if you use a cncShield or if you connect the cable wiring
//  to the ESP32 directly.
#define USE_WITH_CNCSHIELD // or: BARE_ESP32

#if defined(USE_WITH_CNCSHIELD)
#define Z1_DIR_pin_ 16 // digOut -> DIR pin for Z1-axis -> modified-cncShield-for-ESP32: X.DIR -> set to GPIO_16
#define Z2_DIR_pin_ 27 // digOut -> DIR pin for Z2-axis -> modified-cncShield-for-ESP32: Y.DIR -> set to GPIO_27
#define Z3_DIR_pin_ 14 // digOut -> DIR pin for Z3-axis -> modified-cncShield-for-ESP32: Z.DIR -> set to GPIO_14
//                        digOut -> DIR pin for Z4-axis ==> by jumper on cnc-shield bridging ????
//
#define Z1_PUL_pin_ 26 // digOut -> STEP pin for motor z1-axis -> modified-cncShield-for-ESP32: X.STP -> with bare ESP32 use GPIO_26 -> | Direct Port Access:
#define Z2_PUL_pin_ 25 // digOut -> STEP pin for motor z2-axis -> modified-cncShield-for-ESP32: Y.STP -> with bare ESP32 use GPIO_25 -> | Direct Port Access:
#define Z3_PUL_pin_ 17 // digOut -> STEP pin for motor z2-axis -> modified-cncShield-for-ESP32: Z.STP -> with bare ESP32 use GPIO_17 -> | Direct Port Access:
//                         digOut -> STEP pin for motor Z4-axis ==> by jumper on cnc-shield bridging ????
#elif defined(BARE_ESP32)
#else
#endif

// Motor parameters
#define MAX_SPEED 2000   // Maximum motor speed in steps/second
#define ACCELERATION 100 // Maximum acceleration in steps/second^2
#define JERK 50          // Maximum jerk in steps/second^3
// State variables
static int currentPosition = 0; // Motor's current position in steps
static float currentSpeed = 0;  // Motor's current speed in steps/second
static float targetSpeed = 0;   // Motor's target spee in steps/second
// Button state simulation
static int forwardButtonPressed = 0;  // 1 if forward button is pressed, 0 otherwise
static int backwardButtonPressed = 0; // 1 if backward button is pressed, 0 otherwise
static int direction = 0;             // 1 if forward motion, -1 if backward motion (used to add or subtract steps from current position)
// global variables
int btnUpState = 0;
int btnDownState = 0;
unsigned long z_lastMicros;
unsigned long z_stepdelay; // delay time between steps in microns (micro seconds) 1 second = 1000.000 microns
bool isMotionIn_Z = true;
int direction_is_up = 0; // 1 if forward motion, -1 if backward motion, 0 if no motion
bool led_is_on = false;
int z_counter = 300;
int serial_counter = 0;
//
// put function declarations here:
void setup_for_direct_port_manipulation_of_axis_z1_z2_z3(void);
void move_motors_one_step(void);
int calculateStepInterval(float targetSpeed, unsigned long *stepInterval);
void moveToPosition(int targetPosition);
void updateMotorState();
void setButtonState(int forward, int backward);
void getButtonState();
void set_motor_direction();
void set_motor_target_speed();
void set_led_state();

//
// Main
//

void setup()
{

  // initialize the LED pins as output:
  pinMode(LED_BTN_UP_PIN_, OUTPUT);
  pinMode(LED_BTN_DOWN_PIN_, OUTPUT);
  // initialize the pushbutton pins as input:
  pinMode(BTN_UP_PIN_, INPUT);
  pinMode(BTN_DOWN_PIN_, INPUT);
  // initialize the motor direction pins as output:
  pinMode(Z1_DIR_pin_, OUTPUT);
  pinMode(Z2_DIR_pin_, OUTPUT);
  pinMode(Z3_DIR_pin_, OUTPUT);
  //
  digitalWrite(Z1_DIR_pin_, HIGH); // HIGH == to moving in direction UP
  digitalWrite(Z2_DIR_pin_, HIGH);
  digitalWrite(Z3_DIR_pin_, HIGH);
  direction_is_up = true;
  //
  // initially set the motion request to false (no motion of the motors)
  isMotionIn_Z = true;
  //
  // initialize the motor step pins (pulse pins) as output:
  // to do this configure ESP32 for Direct Port Manipulation -> this provides syncronous change of the state of the pins so that all motors step syncronously
  setup_for_direct_port_manipulation_of_axis_z1_z2_z3();
  //
  //
  Serial.begin(9600);
  Serial.println("HI");
  // [z_stepdelay] determines the speed of the motors. The motors used in this project step 200 times/revolution in 1/1 full-step-mode.
  // Formula: delay = (60 seconds * 1000.000) / (200 steps/revolution * n);
  // Explaned: delay = 1 minute (in microseconds) / 200 steps/revolution * number_of_revolutions;
  // -> equals the delay time for the motor to spin 1 full revolution in 1 minute (per minute), when n = 1;
  //
  //  E.g., the motor shall run at 120 rpm -> delay = (60 seconds * 1000.000) / (200 steps/revolution * 120); delay = 2.500;
  //  Or: n = (60 seconds * 1000.000) / (200 steps/revolution * delay);
  z_stepdelay = 1250; // 60 revolutions / minute;
  //

  //
}

void loop()
{
  // INcrement loop-counter for z-motors
  z_counter++;
  serial_counter++;
  // read the state of the pushbutton value:
  if (z_counter % 5000 == 0)
  {
    z_counter = 0;
    getButtonState();
    set_motor_direction();
    set_motor_target_speed();
    set_led_state();
  }

  // Calculate step interval and check if a step is needed
  unsigned long stepInterval;
  if (calculateStepInterval(targetSpeed, &stepInterval))
  {
    // Update current position
    currentPosition += direction;
    // Just temporarily, while move_motors_one_step() is not using the global variable "stepInterval"
    z_stepdelay = stepInterval;
    // move motors one step
    move_motors_one_step();
  }
  else
  {
    if (serial_counter % 15000 == 0)
    {
      serial_counter = 0;
      Serial.print("currentPosition = ");
      Serial.println(currentPosition);
    }
  }

  // //
  // // check if button UP is pressed OR ELSE if button DOWN is pressed, and ELSE turn off both LEDs..
  // // when one of the buttons is pressed set the direction accordingly
  // if (z_counter % 5000 == 0)
  // {
  //   z_counter = 0;
  //   if (btnUpState == HIGH)
  //   {
  //     // turn on LED
  //     if (!led_is_on)
  //     {
  //       led_is_on = true;
  //       digitalWrite(LED_BTN_UP_PIN_, HIGH);
  //       // set the motion request to true
  //       isMotionIn_Z = true;
  //     }
  //     // DEBUG
  //     // Serial.println("BTN UP");
  //     // END DEBUG
  //     // set the motor-driver direction pins for motion UP, if it has not been done already
  //     if (!direction_is_up)
  //     {
  //       direction_is_up = true;
  //       // set the motor-driver directionpins
  //       digitalWrite(Z1_DIR_pin_, HIGH);
  //       digitalWrite(Z2_DIR_pin_, HIGH);
  //       digitalWrite(Z3_DIR_pin_, HIGH);
  //       // initialize the time stamp variable, need to culculate whether the required delay time between steps has elapsed
  //       z_lastMicros = micros();
  //     }
  //   }
  //   else if (btnDownState == HIGH)
  //   {
  //     // turn on LED
  //     if (!led_is_on)
  //     {
  //       led_is_on = true;
  //       digitalWrite(LED_BTN_DOWN_PIN_, HIGH);
  //       // set the motion request to true
  //       isMotionIn_Z = true;
  //     }
  //     // DEBUG
  //     // Serial.println("BTN DOWN");
  //     // END DEBUG
  //     // set the motor-driver direction pins for motion DOWN, if it has not been done already
  //     if (direction_is_up)
  //     {
  //       direction_is_up = false;
  //       // set the motor-driver directionpins
  //       digitalWrite(Z1_DIR_pin_, LOW);
  //       digitalWrite(Z2_DIR_pin_, LOW);
  //       digitalWrite(Z3_DIR_pin_, LOW);
  //       // initialize the time stamp variable, needed to culculate whether the required delay time between steps has elapsed
  //       z_lastMicros = micros();
  //     }
  //   }
  //   else
  //   {
  //     // set the motion request to false
  //     isMotionIn_Z = false;
  //     // trun off LEDs
  //     led_is_on = false;
  //     digitalWrite(LED_BTN_UP_PIN_, LOW);
  //     digitalWrite(LED_BTN_DOWN_PIN_, LOW);
  //     // Serial.println("BTN UP_not pressed");
  //   }
  // }
  // // IF motion is requested..
  // if (isMotionIn_Z)
  // {
  //   move_motors_one_step();
  // }
}

//
void getButtonState()
{
  btnUpState = digitalRead(BTN_UP_PIN_);
  btnDownState = digitalRead(BTN_DOWN_PIN_);
  if (btnUpState == HIGH && btnDownState == LOW)
  {
    forwardButtonPressed = 1;
    backwardButtonPressed = 0;
  }
  if (btnUpState == LOW && btnDownState == HIGH)
  {
    forwardButtonPressed = 0;
    backwardButtonPressed = 1;
  }
  else if (btnUpState == LOW && btnDownState == LOW)
  {
    forwardButtonPressed = 0;
    backwardButtonPressed = 0;
  }
}
//
void set_motor_direction()
{

  // Forward (upward) motion is wanted -> only when the MCU's direction pins are not yet set correctly -> set the pin state
  if (forwardButtonPressed == 1 && backwardButtonPressed == 0 && direction_is_up != 1)
  {
    digitalWrite(Z1_DIR_pin_, HIGH);
    digitalWrite(Z2_DIR_pin_, HIGH);
    digitalWrite(Z3_DIR_pin_, HIGH);
    direction_is_up = 1;
    direction = 1;           // Forward motion
    z_lastMicros = micros(); // Set time stamp needed to perform the first step of the motor
  }
  // Backward (downward) motion is wanted -> only when the MCU's direction pins are not yet set correctly -> set the pin state
  else if (forwardButtonPressed == 0 && backwardButtonPressed == 1 && direction_is_up != -1)
  {
    digitalWrite(Z1_DIR_pin_, LOW);
    digitalWrite(Z2_DIR_pin_, LOW);
    digitalWrite(Z3_DIR_pin_, LOW);
    direction_is_up = -1;
    direction = -1;          // Backward motion
    z_lastMicros = micros(); // Set time stamp needed to perform the first step of the motor
  }
  // No motion is wantend (no button is pressed)
  else if (forwardButtonPressed == 0 && backwardButtonPressed == 0)
  {
    direction_is_up = 0;
  }
}
//
void set_motor_target_speed()
{
  // Determine target speed
  if (forwardButtonPressed && !backwardButtonPressed)
  {
    targetSpeed = MAX_SPEED;
  }
  else if (backwardButtonPressed && !forwardButtonPressed)
  {
    targetSpeed = MAX_SPEED;
  }
  else if (!backwardButtonPressed && !forwardButtonPressed)
  {
    targetSpeed = 0; // Decelerate to stop
  }
}
//
void set_led_state()
{
  // Forward (upward) motion -> turn on LED indicating this direction of motion
  if (forwardButtonPressed == 1 && backwardButtonPressed == 0 && !led_is_on)
  {
    digitalWrite(LED_BTN_UP_PIN_, HIGH);
    led_is_on = true;
  }
  // Backward (downward) motion -> turn on LED indicating this direction of motion
  else if (forwardButtonPressed == 0 && backwardButtonPressed == 1 && !led_is_on)
  {
    digitalWrite(LED_BTN_DOWN_PIN_, HIGH);
    led_is_on = true;
  }
  else if (forwardButtonPressed == 0 && backwardButtonPressed == 0) // turn off both LEDs
  {
    digitalWrite(LED_BTN_UP_PIN_, LOW);
    digitalWrite(LED_BTN_DOWN_PIN_, LOW);
    led_is_on = false;
  }
}

// Place this function inside the setup(); It configures the ESP32 GPIO for direct-port-manipulation, which is used here
// to drive the motors z1, z2 and z3 (and z4) simulataniously.
void setup_for_direct_port_manipulation_of_axis_z1_z2_z3(void)
{
  gpio_config_t io_conf;
  // Configure this obj as output
  io_conf.mode = GPIO_MODE_OUTPUT;
  // Use a bitmask to configure WHICH of the GPIOs shall be used as an output ();
  // Examples of different ways for setting this bit mask:
  // io_conf.pin_bit_mask = 0b10100; // Set GPIO_2 and GPIO_4 as output: 0b10100 -> example for GPIO_2 and GPIO_3: 0b0110;
  // io_conf.pin_bit_mask = (1 << TEST_PIN);
  io_conf.pin_bit_mask = (1 << Z1_PUL_pin_) | (1 << Z2_PUL_pin_) | (1 << Z3_PUL_pin_);
  // io_conf.pin_bit_mask = (1 << 2) | (1 << 4); // this is yet another alternative way to set the bitmask
  // Pass the config obj to the API
  gpio_config(&io_conf);
}

void move_motors_one_step(void)
{
  if (isMotionIn_Z)
  {
    if (micros() - z_lastMicros >= z_stepdelay)
    {
      // PORTD = PORTD & B11110111;  // ARDUINO code for ARDUINO-UNO board --> direct-port-access --> sets port PD3 to LOW
      // digitalWrite(Y1_PUL_pin_, LOW); // arduino-framework --> works fine, but not useful to move two motors simultaniously..
      // digitalWrite(Y2_PUL_pin_, LOW); // arduino-framework --> works fine, but not useful to move two motors simultaniously..
      GPIO.out_w1tc = (1 << Z1_PUL_pin_) | (1 << Z2_PUL_pin_) | (1 << Z3_PUL_pin_); // espressif-api --> direct-port-access --> set pins LOW --> in "w1tc" the c stands for "clear"
      //
      delayMicroseconds(4);
      // PORTD = PORTD | B00001000;  // ARDUIONO code for ARDUINO-UNO board --> direct-port-access--> sets port PD3 to HIGH
      // digitalWrite(Y1_PUL_pin_, HIGH); // arduino-framework --> works fine, but not useful to move two motors simultaniously..
      // digitalWrite(Y2_PUL_pin_, HIGH); // arduino-framework --> works fine, but not useful to move two motors simultaniously..
      GPIO.out_w1ts = (1 << Z1_PUL_pin_) | (1 << Z2_PUL_pin_) | (1 << Z3_PUL_pin_); // espressif-api --> direct-port-access --> set pins HIGH --> in "w1ts" the s stands for "set"
      z_lastMicros = micros();
    }
  }
}

/**
 * Calculate the next step interval for an S-curve motion profile.
 * @param targetSpeed: Desired target speed in steps/second.
 * @param stepInterval: Time between steps in microseconds (output parameter).
 * @return 1 if the motor needs to step, 0 otherwise.
 */
int calculateStepInterval(float targetSpeed, unsigned long *stepInterval)
{
  static float acceleration = 0; // Current acceleration in steps/second^2
  static float jerk = JERK;      // Current jerk in steps/second^3
  bool is_accelerating = false;

  // Adjust acceleration smoothly
  if (currentSpeed < targetSpeed)
  {
    is_accelerating = true;
    acceleration += jerk * 0.01; // Increase acceleration
    if (acceleration > ACCELERATION)
      acceleration = ACCELERATION;
  }
  // Decelerate smoothly
  else if (currentSpeed > targetSpeed)
  {
    acceleration -= jerk * 0.01; // Decrease acceleration
    if (acceleration < -ACCELERATION)
      acceleration = -ACCELERATION;
  }
  // Adjust speed smoothly
  currentSpeed += acceleration * 0.01;
  if (currentSpeed > targetSpeed && targetSpeed != 0)
    currentSpeed = targetSpeed;
  if (currentSpeed < 0)
    currentSpeed = 0;

  // Calculate step interval
  if (currentSpeed > 0)
  {
    *stepInterval = (unsigned long)(1e6 / currentSpeed); // Microseconds per step
    return 1;                                            // Motor needs to step
  }

  return 0; // Motor does not need to step
}

/**
 * Move the stepper motor to a target position with S-curve motion.
 * @param targetPosition: Target position in motor steps.
 */
void moveToPosition(int targetPosition)
{
  unsigned long stepInterval;
  float targetSpeed = 0;
  int direction = (targetPosition > currentPosition) ? 1 : -1;

  while (currentPosition != targetPosition)
  {
    // Determine target speed based on remaining distance
    int remainingSteps = abs(targetPosition - currentPosition);
    if (remainingSteps > MAX_SPEED / ACCELERATION)
    {
      targetSpeed = MAX_SPEED; // Cruise speed
    }
    else
    {
      targetSpeed = sqrt(2.0 * ACCELERATION * remainingSteps); // Deceleration phase
    }

    // Calculate step interval and check if a step is needed
    if (calculateStepInterval(targetSpeed, &stepInterval))
    {
      // Simulate a step
      currentPosition += direction;
      printf("Step to position: %d\n", currentPosition);
      usleep(stepInterval); // Delay for the step interval
    }
  }

  // Stop motor
  currentSpeed = 0;
}

/**
 * Update motor state based on button presses.
 * This function should be called in a loop to handle button presses and motion.
 */
void updateMotorState()
{
  unsigned long stepInterval;
  float targetSpeed = 0;

  // Determine direction and target speed
  int direction = 0;
  if (forwardButtonPressed && !backwardButtonPressed)
  {
    direction = 1; // Forward motion
    targetSpeed = MAX_SPEED;
  }
  else if (backwardButtonPressed && !forwardButtonPressed)
  {
    direction = -1; // Backward motion
    targetSpeed = MAX_SPEED;
  }
  else
  {
    targetSpeed = 0; // Decelerate to stop
  }

  // Calculate step interval and check if a step is needed
  if (calculateStepInterval(targetSpeed, &stepInterval))
  {
    // Simulate a step
    currentPosition += direction;
    printf("Step to position: %d\n", currentPosition);
    usleep(stepInterval); // Delay for the step interval
  }
}

/**
 * Get the current position of the stepper motor.
 * @return Current position in motor steps.
 */
int getCurrentPosition()
{
  return currentPosition;
}

/**
 * Simulate pressing or releasing buttons.
 * @param forward: 1 to press forward button, 0 to release.
 * @param backward: 1 to press backward button, 0 to release.
 */
void setButtonState(int forward, int backward)
{
  forwardButtonPressed = forward;
  backwardButtonPressed = backward;
}