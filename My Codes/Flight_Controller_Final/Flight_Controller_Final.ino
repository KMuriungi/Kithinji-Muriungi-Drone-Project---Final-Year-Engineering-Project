/*ECE 590: ENGINEERING PROJECT II
 * MULTI-PURPOSE QUADCOPTER
 * : Arduino Based PID Controller
 * EC/09/14
 * KITHINJI MURIUNGI
 * 2018/19
 * Supervisor: Mr. S.B.Kifalu
*/
/*
 ************************************************************
 *-----------------HARDWARE / CIRCUIT WIRING----------------*
 ************************************************************
 *Motors:(Purple)   |   Channels: (White)   |   LED -> 13                       |   MPU6050 
 *I -> 4            |   I -> 8              |   VCC -> Red                      |   SDA -> A4 (White)
 *II -> 6           |   II ->9              |   GND -> Green                    |   SCL -> A5 (Purple)
 *III -> 7          |   III ->10            |   Vin -> White                    |   Vcc -> Red (5V)
 *IV -> 5           |   IV -> 11            |   Batt Monitor -> A0 (Yellow)     |
*/
/*
 ************************************************************
 *----------------------DECLARATION-------------------------*
 ************************************************************
*/
#include <Wire.h> // Library to enable MPU6050 configurations and calibrations
#define CHANNEL1 0 //Pin 8 : Roll on Mode 2
#define CHANNEL2 1 //Pin 9 : Pitch on Mode 2
#define CHANNEL3 2 //Pin 10 : Throttle on Mode 2
#define CHANNEL4 3 //Pin 11 : Yaw on Mode 2

#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

#define X           0     // X axis
#define Y           1     // Y axis
#define Z           2     // Z axis
#define MPU_ADDRESS 0x68  // I2C address of the MPU-6050
#define FREQ        250   // Sampling frequency ///Equivalent to 4000us : (1/250)
#define SSF_GYRO    65.5  // Sensitivity Scale Factor of the gyro from datasheet

#define STOPPED  0
#define STARTING 1
#define STARTED  2
// ---------------- Receiver variables ---------------------------------------
/**
 * Received flight instructions formatted with good units, in that order : [Yaw, Pitch, Roll, Throttle]
 * Units:
 *  - Yaw      : degree/sec
 *  - Pitch    : degree
 *  - Roll     : degree
 *  - Throttle : µs  
 */
float instruction[4];

// Previous state of each channel (HIGH or LOW)
volatile byte previous_state[4];

// Duration of the pulse on each channel of the receiver in µs (must be within 1000µs & 2000µs)
volatile unsigned int pulse_length[4] = {1500, 1500, 1000, 1500};

// Used to calculate pulse duration on each channel
volatile unsigned long current_time;
volatile unsigned long timer[4]; // Timer of each channel

// Used to configure which control (yaw, pitch, roll, throttle) is on which channel
int mode_mapping[4];
// ----------------------- MPU variables -------------------------------------
// The RAW values got from gyro (in °/sec) in that order: X, Y, Z
int gyro_raw[3] = {0,0,0};

// Average gyro offsets of each axis in that order: X, Y, Z
long gyro_offset[3] = {0, 0, 0};

// Calculated angles from gyro's values in that order: X, Y, Z
float gyro_angle[3]  = {0,0,0};

// The RAW values got from accelerometer (in m/sec²) in that order: X, Y, Z
int acc_raw[3] = {0 ,0 ,0};

// Calculated angles from accelerometer's values in that order: X, Y, Z
float acc_angle[3] = {0,0,0};

// Total 3D acceleration vector in m/s²
long acc_total_vector;

/**
 * Real measures on 3 axis calculated from gyro AND accelerometer in that order : Yaw, Pitch, Roll
 *  - Left wing up implies a positive roll
 *  - Nose up implies a positive pitch
 *  - Nose right implies a positive yaw
 */
float measures[3] = {0, 0, 0};

// MPU's temperature
int temperature;

// Init flag set to TRUE after first loop
boolean initialized;
// ----------------------- Variables for servo signal generation -------------
unsigned int  period; // Sampling period
unsigned long loop_timer;
unsigned long now, difference;

unsigned long pulse_length_esc1 = 1000,
        pulse_length_esc2 = 1000,
        pulse_length_esc3 = 1000,
        pulse_length_esc4 = 1000;

// ------------- Global variables used for PID controller --------------------
float errors[3];                     // Measured errors (compared to instructions) : [Yaw, Pitch, Roll]
float error_sum[3]      = {0, 0, 0}; // Error sums (used for integral component) : [Yaw, Pitch, Roll]
float previous_error[3] = {0, 0, 0}; // Last errors (used for derivative component) : [Yaw, Pitch, Roll]
// ---------------------------------------------------------------------------
/**
 * Status of the quadcopter:
 *   -> 0 : stopped
 *   -> 1 : starting
 *   -> 2 : started
 */
int status = STOPPED;
// --------------------------------------------------------
int battery_voltage;
// --------------------------------------------------------

/*
 ************************************************************
 *--------------------INITIALIZATION------------------------*
 ************************************************************
*/
void setup() {
    //Serial.begin(115200);
    
    // Start I2C communication
    
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

    // Turn LED on during setup
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    // Set pins #4 #5 #6 #7 as outputs
    DDRD |= B11110000;

    setupMpu6050Registers();

    calibrateMpu6050();

    configureChannelMapping();

    // Configure interrupts for receiver
    //NO DigitalRead or DigitalWrite due to memory and unnecessary delays in the program
    PCICR  |= (1 << PCIE0);  // Set PCIE0 to enable PCMSK0 scan
    PCMSK0 |= (1 << PCINT0); // Set PCINT0 (digital input 8) to trigger an interrupt on state change
    PCMSK0 |= (1 << PCINT1); // Set PCINT1 (digital input 9) to trigger an interrupt on state change
    PCMSK0 |= (1 << PCINT2); // Set PCINT2 (digital input 10)to trigger an interrupt on state change
    PCMSK0 |= (1 << PCINT3); // Set PCINT3 (digital input 11)to trigger an interrupt on state change

    period = (1000000/FREQ) ; // Sampling period in µs

    // Initialize loop_timer
    loop_timer = micros();

    // Turn LED off now setup is done
    digitalWrite(13, LOW); // To monitor the status especially the initial configs : Arming, Battery, Config
}

/*
 ************************************************************
 *------------------MAIN PROGRAM LOOP-----------------------*
 ************************************************************
*/
void loop() {
    // 1. First, read raw values from MPU-6050
    readSensor();

    // 2. Calculate angles from gyro & accelerometer's values
    calculateAngles();

    // 3. Translate received data into usable values
    getFlightInstruction();

    // 4. Calculate errors comparing received instruction with measures
    calculateErrors();

    if (isStarted()) {
        // 5. Calculate motors speed with PID controller
        pidController();

        compensateBatteryDrop(); //Ensures safe flight with enough power: Failure to which it will shutdown motors 
    }

    // 6. Apply motors speed
    applyMotorSpeed(); // Apply motor speeds accordingly as per the command: Yaw,Pitch,Roll,Throttle
}/*
 ************************************************************
 *----------------------END MAIN LOOP-------------------------*
 ************************************************************
*/

/**
 * Generate servo-signal on digital pins #4 #5 #6 #7 with a frequency of 250Hz (4ms period).
 * Direct port manipulation is used for performances.... Without this instant/realtime control isn't possible. This could cause delays in commands.
 *
 * This function might not take more than 2ms to run, which lets 2ms remaining to do other stuff. ----THIS IS THE SOLE REASON OF ADDRESSING REGISTERS instead of ordinary ports
 *https:// www.arduino.cc/en/Reference/PortManipulation 
 */

 /*
 **********************************************************************************************
 *--------------------------------------CUSTOM FUNCTIONS--------------------------------------*
 **********************************************************************************************
*/

/*
 ************************************************************
 *---------------MOTOR CONTROL FUNCTION---------------------*
 ************************************************************
*/
void applyMotorSpeed() {
    // Refresh rate is 250Hz: send ESC pulses every 4000µs
    while ((now = micros()) - loop_timer < period);

    // Update loop timer
    loop_timer = now;

    // Set pins #4 #5 #6 #7 HIGH
    PORTD |= B11110000;

    // Wait until all pins #4 #5 #6 #7 are LOW
    while (PORTD >= 16) {
        now        = micros();
        difference = now - loop_timer;

        if (difference >= pulse_length_esc1) PORTD &= B11101111; // Set pin #4 LOW
        if (difference >= pulse_length_esc2) PORTD &= B11011111; // Set pin #5 LOW
        if (difference >= pulse_length_esc3) PORTD &= B10111111; // Set pin #6 LOW
        if (difference >= pulse_length_esc4) PORTD &= B01111111; // Set pin #7 LOW
    }
}

 /*
 ************************************************************
 *------------------MPU6050 READ FUNCTION-------------------*
 ************************************************************
*/
//Requesting raw values from MPU6050.
void readSensor() {
    Wire.beginTransmission(MPU_ADDRESS); // Start communicating with the MPU-6050
    Wire.write(0x3B);                    // Send the requested starting register
    Wire.endTransmission();              // End the transmission
    Wire.requestFrom(MPU_ADDRESS,14);    // Request 14 bytes from the MPU-6050

    // Wait until all the bytes are received
    while(Wire.available() < 14);

    acc_raw[X]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[X] variable
    acc_raw[Y]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[Y] variable
    acc_raw[Z]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[Z] variable
    temperature = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the temperature variable
    gyro_raw[X] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyro_raw[X] variable
    gyro_raw[Y] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyro_raw[Y] variable
    gyro_raw[Z] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyro_raw[Z] variable
}

/*
 ************************************************************
 *---------------CALCULATING ANGLES FUNCTION----------------*
 ************************************************************
*/
//Calculate real angles from gyro and accelerometer's values
void calculateAngles()
{
    calculateGyroAngles();
    calculateAccelerometerAngles();

    if (initialized) {
        // Correct the drift of the gyro with the accelerometer
        //These can be changed accordingly based on the accuracy of the Sensor used
        gyro_angle[X] = gyro_angle[X] * 0.9996 + acc_angle[X] * 0.0004;
        gyro_angle[Y] = gyro_angle[Y] * 0.9996 + acc_angle[Y] * 0.0004;
    } else {
        // At very first start, init gyro angles with accelerometer angles
        resetGyroAngles();

        initialized = true;
    }

    // To dampen the pitch and roll angles a complementary filter is used
    //Software inplementation of Complimentary Filter: HPF & LPF 
    //Gyro -- Integral -- HPF
    //Accelerometer -- Angle Conversion -- LPF
    measures[ROLL]  = measures[ROLL]  * 0.9 + gyro_angle[X] * 0.1;
    measures[PITCH] = measures[PITCH] * 0.9 + gyro_angle[Y] * 0.1;
    measures[YAW]   = -gyro_raw[Z] / SSF_GYRO; // Store the angular motion for this axis
}

/*
 ************************************************************
 *------------CALCULATE GYRO ANGLES ONLY FUNCTION-----------*
 ************************************************************
*/
//Calculate pitch & roll angles using only the gyro.
void calculateGyroAngles()
{
    // Subtract offsets
    gyro_raw[X] -= gyro_offset[X];
    gyro_raw[Y] -= gyro_offset[Y];
    gyro_raw[Z] -= gyro_offset[Z];

    // Angle calculation using integration
    gyro_angle[X] += (gyro_raw[X] / (FREQ * SSF_GYRO));
    gyro_angle[Y] += (-gyro_raw[Y] / (FREQ * SSF_GYRO)); // Change sign to match the accelerometer's one

    // Transfer roll to pitch if IMU has yawed
    gyro_angle[Y] += gyro_angle[X] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
    gyro_angle[X] -= gyro_angle[Y] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
}

/*
 ************************************************************
 *------CALCULATE ACCELEROMETER ANGLES ONLY FUNCTION--------*
 ************************************************************
*/
//Calculate pitch & roll angles using only the accelerometer.

void calculateAccelerometerAngles()
{
    // Calculate total 3D acceleration vector : √(X² + Y² + Z²)
    acc_total_vector = sqrt(pow(acc_raw[X], 2) + pow(acc_raw[Y], 2) + pow(acc_raw[Z], 2));

    // To prevent asin to produce a NaN, make sure the input value is within [-1;+1]
    if (abs(acc_raw[X]) < acc_total_vector) {
        acc_angle[X] = asin((float)acc_raw[Y] / acc_total_vector) * (180 / PI); // asin gives angle in radian. Convert to degree multiplying by 180/pi
    }

    if (abs(acc_raw[Y]) < acc_total_vector) {
        acc_angle[Y] = asin((float)acc_raw[X] / acc_total_vector) * (180 / PI);
    }
}

/**
 *Motors and MPU6050 Configurations
 *
 * (A) (B)     x
 *   \ /     z ↑
 *    X       \|
 *   / \       +----→ y
 * (C) (D)
 *
 *X = Roll
 *Y = Pitch
 *Z = Yaw
 *
 * Motors A & D run clockwise.
 * Motors B & C run counter-clockwise.
 *
 * Each motor output value range is about 1000µs to 2000µs
 */

/*
 ************************************************************
 *----------------PID CONTROLLER FUNCTION-------------------*
 ************************************************************
*/
 
void pidController() {
  //These are the Gain Values that are tuned accordingly
  //Every slight change in the quadcopter will require tuning.
    float Kp[3]        = {3, 1.3, 1.3};               // P coefficients : Yaw, Pitch, Roll
    float Ki[3]        = {0.045, 0.03, 0.03};        // I coefficients : Yaw, Pitch, Roll
    float Kd[3]        = {0, 15, 15};                 // D coefficients : Yaw, Pitch, Roll
    float delta_err[3] = {0, 0, 0};                // Error deltas   : Yaw, Pitch, Roll
    float yaw_pid      = 0;
    float pitch_pid    = 0;
    float roll_pid     = 0;

    // Initialize motor commands with throttle
    pulse_length_esc1 = instruction[THROTTLE];
    pulse_length_esc2 = instruction[THROTTLE];
    pulse_length_esc3 = instruction[THROTTLE];
    pulse_length_esc4 = instruction[THROTTLE];

    // Do not calculate anything if throttle is 0

    //PID Calculations
    
    if (instruction[THROTTLE] >= 1012) {
        // Calculate sum of errors : Integral coefficients
        error_sum[YAW]   += errors[YAW];
        error_sum[PITCH] += errors[PITCH];
        error_sum[ROLL]  += errors[ROLL];

        // Calculate error delta : Derivative coefficients
        delta_err[YAW]   = errors[YAW]   - previous_error[YAW];
        delta_err[PITCH] = errors[PITCH] - previous_error[PITCH];
        delta_err[ROLL]  = errors[ROLL]  - previous_error[ROLL];

        // Save current error as previous_error for next time
        previous_error[YAW]   = errors[YAW];
        previous_error[PITCH] = errors[PITCH];
        previous_error[ROLL]  = errors[ROLL];

        // PID = e.Kp + ∫e.Ki + Δe.Kd    ---- PID MODEL---- 
        //This is the final calculated PID Model value
        yaw_pid   = (errors[YAW]   * Kp[YAW])   + (error_sum[YAW]   * Ki[YAW])   + (delta_err[YAW]   * Kd[YAW]);
        pitch_pid = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (delta_err[PITCH] * Kd[PITCH]);
        roll_pid  = (errors[ROLL]  * Kp[ROLL])  + (error_sum[ROLL]  * Ki[ROLL])  + (delta_err[ROLL]  * Kd[ROLL]);

        // Calculate pulse duration for each ESC
        //Due to the nature of the PID, the pulse length of any ESC does not matter.
        pulse_length_esc1 = instruction[THROTTLE] + roll_pid + pitch_pid - yaw_pid;
        pulse_length_esc2 = instruction[THROTTLE] - roll_pid + pitch_pid + yaw_pid;
        pulse_length_esc3 = instruction[THROTTLE] + roll_pid - pitch_pid + yaw_pid;
        pulse_length_esc4 = instruction[THROTTLE] - roll_pid - pitch_pid - yaw_pid;
    }
    //The minimum value helps in having the lowest speed and torque just to indicate that the rotors are spining correctly and the drone is fully armed  
    pulse_length_esc1 = minMax(pulse_length_esc1, 1100, 2000);  //Min-Max Mapping Accordingly 1100/1000 - 2000
    pulse_length_esc2 = minMax(pulse_length_esc2, 1100, 2000);  
    pulse_length_esc3 = minMax(pulse_length_esc3, 1100, 2000);
    pulse_length_esc4 = minMax(pulse_length_esc4, 1100, 2000);
}

/*
 ************************************************************
 *----------------ERROR CALCULATION FUNCTION----------------*
 ************************************************************
*/
//Calculate errors of Yaw, Pitch & Roll: this is simply the difference between the measure and the command.
void calculateErrors() {
  //This facilitates in Closed-Loop Feedback system 
    errors[YAW]   = instruction[YAW]   - measures[YAW];
    errors[PITCH] = instruction[PITCH] - measures[PITCH];
    errors[ROLL]  = instruction[ROLL]  - measures[ROLL];
}

/**
 * Calculate real value of flight instructions from pulses length of each channel.
 *
 * - Roll     : from -33° to 33°
 * - Pitch    : from -33° to 33°
 * - Yaw      : from -180°/sec to 180°/sec
 * - Throttle : from 1000µs to 1800µs
 */

 /*
 ************************************************************
 *----------------FLIGHT INSTRUCTIONS FUNCTION--------------*
 ************************************************************
*/
void getFlightInstruction() {
    instruction[YAW]      = map(pulse_length[mode_mapping[YAW]], 1000, 2000, -180, 180);
    instruction[PITCH]    = map(pulse_length[mode_mapping[PITCH]], 1000, 2000, 33, -33); //Since the operates on "Reverse Mode"
    instruction[ROLL]     = map(pulse_length[mode_mapping[ROLL]], 1000, 2000, -33, 33);
    instruction[THROTTLE] = map(pulse_length[mode_mapping[THROTTLE]], 1000, 2000, 1000, 1800); // Get some room to keep control at full speed
}

/**
 * Customize mapping of controls: set here which command is on which channel and call
 * this function in setup() routine.
 */

 /*
 ************************************************************
 *----------------CHANNEL MAPPING FUNCTION------------------*
 ************************************************************
*/
void configureChannelMapping() {
  //Mapping according to the TRANSMITTER Configuration
  //Using these mapping,confirm that the Transmitter is set to MODE 2 (LEFT: Throttle & Yaw ---- RIGHT: Pitch & Roll) 
    mode_mapping[YAW]      = CHANNEL4;
    mode_mapping[PITCH]    = CHANNEL2;
    mode_mapping[ROLL]     = CHANNEL1;
    mode_mapping[THROTTLE] = CHANNEL3;
}

/*
 ************************************************************
 *------------MPU6050 CONFIGURATION FUNCTION----------------*
 ************************************************************
*/

/**
 * Configure gyro and accelerometer precision as following:
 * Check from the MPU6050 Datasheet
 *  - accelerometer: ±8g
 *  - gyro: ±500°/s
 */
void setupMpu6050Registers() {
    // Configure power management
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(0x6B);                    // Request the PWR_MGMT_1 register
    Wire.write(0x00);                    // Apply the desired configuration to the register
    Wire.endTransmission();              // End the transmission
  
    // Configure the gyro's sensitivity
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(0x1B);                    // Request the GYRO_CONFIG register
    Wire.write(0x08);                    // Apply the desired configuration to the register : ±500°/s
    Wire.endTransmission();              // End the transmission
  
    // Configure the acceleromter's sensitivity
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(0x1C);                    // Request the ACCEL_CONFIG register
    Wire.write(0x10);                    // Apply the desired configuration to the register : ±8g
    Wire.endTransmission();              // End the transmission
  
    // Configure low pass filter
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(0x1A);                    // Request the CONFIG register
    Wire.write(0x03);                    // Set Digital Low Pass Filter about ~43Hz
    Wire.endTransmission();              // End the transmission
}

/*
 ************************************************************
 *---------------MPU6050 CALIBRATION FUNCTION---------------*
 ************************************************************
*/

/**
 * Calibrate MPU6050: take 2000 samples to calculate average offsets.
 * During this step, the quadcopter needs to be static and on a horizontal surface.
 *
 * This function also sends low throttle signal to each ESC to init and prevent them beeping annoyingly.
 *
 * This function might take ~2sec for 2000 samples.
 */
void calibrateMpu6050()
{
    // This takes place at the initial stage when the Quad is first tuned ON.
    //This is Indicated by the continuous lighting of the GREEN LED.
    //When the GREEN LED Goes off, it indicates that the configurations are over.
    int max_samples = 2000;

    for (int i = 0; i < max_samples; i++) {
        readSensor();

        gyro_offset[X] += gyro_raw[X];
        gyro_offset[Y] += gyro_raw[Y];
        gyro_offset[Z] += gyro_raw[Z];

        // Generate low throttle pulse to init ESC and prevent them beeping
        PORTD |= B11110000;                  // Set pins #4 #5 #6 #7 HIGH
        delayMicroseconds(1000); // Wait 1000µs
        PORTD &= B00001111;                  // Then set LOW

        // Just wait a bit before next loop
        delay(3);
    }

    // Calculate average offsets
    gyro_offset[X] /= max_samples;
    gyro_offset[Y] /= max_samples;
    gyro_offset[Z] /= max_samples;
}

/**
 * Make sure that given value is not over min_value/max_value range.
 * @param float value     : The value to convert
 * @param float min_value : The min value
 * @param float max_value : The max value
 */
float minMax(float value, float min_value, float max_value) {
    if (value > max_value) {
        value = max_value;
    } else if (value < min_value) {
        value = min_value;
    }

    return value;
}

/*
 ************************************************************
 *--------------WHEN QUAD IS STARTED FUNCTION---------------*
 ************************************************************
*/

/**
 * Return whether the quadcopter is started.
 * To start the quadcopter, move the left stick in bottom left corner then, move it back in center position.
 * To stop the quadcopter move the left stick in bottom right corner.
 */
bool isStarted()
{
    // When left stick is moved in the bottom left corner
    if (status == STOPPED && pulse_length[mode_mapping[YAW]] <= 1012 && pulse_length[mode_mapping[THROTTLE]] <= 1012) {
        status = STARTING;
    }

    // When left stick is moved back in the center position
    if (status == STARTING && pulse_length[mode_mapping[YAW]] == 1500 && pulse_length[mode_mapping[THROTTLE]] <= 1012) {
        status = STARTED;

        // Reset PID controller's variables to prevent confusion during the start
        resetPidController();

        resetGyroAngles();
    }

    // When left stick is moved in the bottom right corner
    if (status == STARTED && pulse_length[mode_mapping[YAW]] >= 1988 && pulse_length[mode_mapping[THROTTLE]] <= 1012) {
        status = STOPPED;
        // Make sure to always stop motors when status is STOPPED
        stopAll();
    }

    return status == STARTED;
}

/*
 ************************************************************
 *----------------RESET GYRO ANGLES FUNCTION----------------*
 ************************************************************
*/
//Reset gyro's angles with accelerometer's angles.
void resetGyroAngles()
{
    gyro_angle[X] = acc_angle[X];
    gyro_angle[Y] = acc_angle[Y];
}

/*
 ************************************************************
 *---------------STOPPING ALL MOTOR FUNCTION----------------*
 ************************************************************
*/
//Reset motors' pulse length to 1000µs to totally stop them.
void stopAll()
{
    pulse_length_esc1 = 1000;
    pulse_length_esc2 = 1000;
    pulse_length_esc3 = 1000;
    pulse_length_esc4 = 1000;
}

/*
 ************************************************************
 *-------------RESET PID CONTROLLER FUNCTION----------------*
 ************************************************************
*/
//Reset all PID controller's variables.
void resetPidController()
{
    errors[YAW]   = 0;
    errors[PITCH] = 0;
    errors[ROLL]  = 0;

    error_sum[YAW]   = 0;
    error_sum[PITCH] = 0;
    error_sum[ROLL]  = 0;

    previous_error[YAW]   = 0;
    previous_error[PITCH] = 0;
    previous_error[ROLL]  = 0;
}

/*
 ************************************************************
 *------------BATTERY COMPENSATION FUNCTION-----------------*
 ************************************************************
*/
//Compensate battery drop applying a coefficient on output values
void compensateBatteryDrop() {
    if (isBatteryConnected()) {
        pulse_length_esc1 += pulse_length_esc1 * ((1240 - battery_voltage) / (float) 3500);
        pulse_length_esc2 += pulse_length_esc2 * ((1240 - battery_voltage) / (float) 3500);
        pulse_length_esc3 += pulse_length_esc3 * ((1240 - battery_voltage) / (float) 3500);
        pulse_length_esc4 += pulse_length_esc4 * ((1240 - battery_voltage) / (float) 3500);
    }
}

/*
 ************************************************************
 *--------------READ BATTERY VOLTAGE FUNCTION---------------*
 ************************************************************
*/
//Read battery voltage & return whether the battery seems connected
bool isBatteryConnected() {
    // A complementary filter is used to reduce noise.
    // 0.09853 = 0.08 * 1.2317.
    battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

    return battery_voltage < 1240 && battery_voltage > 800;
}

/*
 ************************************************************************
 *---INTERRUPTS TO ENABLE TRANSMITTER-RECEIVER-CONTROLLER COMMUNICATE---*
 ************************************************************************
*/

/**
 * This Interrupt Sub Routine is called each time input 8, 9, 10 or 11 changed state.
 * Read the receiver signals in order to get flight instructions.
 * This routine must be as fast as possible to prevent main program to be messed up.
 * The trick here is to use port registers to read pin state.
 * It is less convenient but more efficient, which is the most important here.
 */
ISR(PCINT0_vect) {
    current_time = micros();

    // Channel 1 -------------------------------------------------
    if (PINB & B00000001) {    // Is input 8 high ?
      //Doing (PINB & B00000001) is the same as digitalRead(8) with the advantage of using less CPU loops.
        if (previous_state[CHANNEL1] == LOW) {                     // Input 8 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL1] = HIGH;                       // Save current state
            timer[CHANNEL1] = current_time;                        // Save current time
        }
    } else if (previous_state[CHANNEL1] == HIGH) {                 // Input 8 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL1] = LOW;                            // Save current state
        pulse_length[CHANNEL1] = current_time - timer[CHANNEL1];   // Calculate pulse duration & save it
    }

    // Channel 2 -------------------------------------------------
    if (PINB & B00000010) {                                        // Is input 9 high ?
      //Doing (PINB & B00000001) is the same as digitalRead(9) with the advantage of using less CPU loops.
        if (previous_state[CHANNEL2] == LOW) {                     // Input 9 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL2] = HIGH;                       // Save current state
            timer[CHANNEL2] = current_time;                        // Save current time
        }
    } else if (previous_state[CHANNEL2] == HIGH) {                 // Input 9 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL2] = LOW;                            // Save current state
        pulse_length[CHANNEL2] = current_time - timer[CHANNEL2];   // Calculate pulse duration & save it
    }

    // Channel 3 -------------------------------------------------
    if (PINB & B00000100) {                                        // Is input 10 high ?
      //Doing (PINB & B00000001) is the same as digitalRead(10) with the advantage of using less CPU loops.
        if (previous_state[CHANNEL3] == LOW) {                     // Input 10 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL3] = HIGH;                       // Save current state
            timer[CHANNEL3] = current_time;                        // Save current time
        }
    } else if (previous_state[CHANNEL3] == HIGH) {                 // Input 10 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL3] = LOW;                            // Save current state
        pulse_length[CHANNEL3] = current_time - timer[CHANNEL3];   // Calculate pulse duration & save it
    }

    // Channel 4 -------------------------------------------------
    if (PINB & B00001000) {                                        // Is input 11 high ?
      //Doing (PINB & B00000001) is the same as digitalRead(11) with the advantage of using less CPU loops.
        if (previous_state[CHANNEL4] == LOW) {                     // Input 11 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL4] = HIGH;                       // Save current state
            timer[CHANNEL4] = current_time;                        // Save current time
        }
    } else if (previous_state[CHANNEL4] == HIGH) {                 // Input 11 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL4] = LOW;                            // Save current state
        pulse_length[CHANNEL4] = current_time - timer[CHANNEL4];   // Calculate pulse duration & save it
    }
}/*END OF THE CODE
  * 
  *-------------------------------------------* 
  *--------------Signature--------------------*
  *-------------------------------------------*
  *
  *MULTI-PURPOSE QUADCOPTER
  *:Arduino-Based PID  Controller
  *FINAL YEAR ENGINEERING
  *ECE 590 : ENGINEERING PROJECT II
  *KITHINJI MURIUNGI
  *EC/09/14
  *2018/19
  *Supervised By: Mr. S. B. Kifalu
   */
 
