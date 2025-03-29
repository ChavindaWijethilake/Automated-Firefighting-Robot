# Arduino Code for a Line Following Robot with Obstacle Detection and Servo/Pump Control

This Arduino code is designed for a line following robot that can also detect obstacles using three infrared (IR) sensors (right, front, left). Upon detecting an obstacle, the robot stops, activates a pump, and sweeps a servo motor. The robot resumes line following if no obstacle is detected.

## Hardware Connections

The code defines the following pin assignments:

**Motor Control (L298N Motor Driver):**

* `enA`: Pin 10 (Enable for Motor A) - Connected to the ENA pin of the L298N.
* `in1`: Pin 9 (Input 1 for Motor A) - Connected to the IN1 pin of the L298N.
* `in2`: Pin 8 (Input 2 for Motor A) - Connected to the IN2 pin of the L298N.
* `in3`: Pin 7 (Input 3 for Motor B) - Connected to the IN3 pin of the L298N.
* `in4`: Pin 6 (Input 4 for Motor B) - Connected to the IN4 pin of the L298N.
* `enB`: Pin 5 (Enable for Motor B) - Connected to the ENB pin of the L298N.

**IR Sensors (Analog Inputs):**

* `ir_R`: Analog Pin A0 (IR Sensor Right) - Connected to the analog output of the right IR sensor.
* `ir_F`: Analog Pin A1 (IR Sensor Front) - Connected to the analog output of the front IR sensor.
* `ir_L`: Analog Pin A2 (IR Sensor Left) - Connected to the analog output of the left IR sensor.

**Servo Motor (PWM Output):**

* `servo`: Analog Pin A4 (Servo Motor Signal) - Connected to the signal pin of the servo motor.

**Pump (Digital Output):**

* `pump`: Analog Pin A5 (Pump Control) - Connected to the control pin of the pump (likely through a relay or transistor).

## Code Description

### Definitions

* `enA`, `in1`, `in2`, `in3`, `in4`, `enB`: Define the digital pins connected to the motor driver for controlling two DC motors.
* `ir_R`, `ir_F`, `ir_L`: Define the analog pins connected to the right, front, and left IR obstacle detection sensors.
* `servo`: Defines the analog pin connected to the servo motor.
* `pump`: Defines the analog pin connected to control the pump.
* `Speed`: An integer variable set to 160, controlling the speed of the motors (used in `analogWrite`).
* `s1`, `s2`, `s3`: Integer variables to store the analog readings from the right, front, and left IR sensors, respectively.

### `setup()` Function

* Initializes serial communication at a baud rate of 9600 for debugging and sensor value monitoring.
* Sets the pin modes for all defined pins:
    * IR sensor pins (`ir_R`, `ir_F`, `ir_L`) are set as `INPUT`.
    * Motor control pins (`enA`, `in1`, `in2`, `in3`, `in4`, `enB`) are set as `OUTPUT`.
    * Servo motor pin (`servo`) is set as `OUTPUT`.
    * Pump control pin (`pump`) is set as `OUTPUT`.
* Sweeps the servo motor through a series of angles (90 to 140, 140 to 40, and 40 to 95 degrees) using the `servoPulse()` function. This is likely a calibration or initial positioning sequence for the servo.
* Sets the initial speed of both motors using `analogWrite()` on the enable pins (`enA`, `enB`).
* Introduces a 500-millisecond delay.

### `loop()` Function

* Reads the analog values from the right (`ir_R`), front (`ir_F`), and left (`ir_L`) IR sensors and stores them in `s1`, `s2`, and `s3` respectively.
* Prints the sensor values to the Serial Monitor for debugging.
* Introduces a 50-millisecond delay.
* **Obstacle Detection and Response:**
    * If the right IR sensor (`s1`) reads a value below 250 (indicating a close object):
        * Stops the motors using `Stop()`.
        * Turns the pump ON by setting `digitalWrite(pump, 1)`.
        * Sweeps the servo motor from 90 to 40 degrees and then back to 90 degrees.
    * Else if the front IR sensor (`s2`) reads a value below 350:
        * Stops the motors.
        * Turns the pump ON.
        * Sweeps the servo motor from 90 to 140 degrees, then to 40 degrees, and finally back to 90 degrees.
    * Else if the left IR sensor (`s3`) reads a value below 250:
        * Stops the motors.
        * Turns the pump ON.
        * Sweeps the servo motor from 90 to 140 degrees and then back to 90 degrees.
* **Line Following Logic (if no immediate obstacle is detected):**
    * Else if the right IR sensor (`s1`) reads a value between 251 and 700 (likely indicating the line is to the right):
        * Turns the pump OFF (`digitalWrite(pump, 0)`).
        * Moves the robot backward for a short duration (`backword()`, `delay(100)`).
        * Turns the robot right for a short duration (`turnRight()`, `delay(200)`).
    * Else if the front IR sensor (`s2`) reads a value between 251 and 800 (likely indicating the line is straight ahead):
        * Turns the pump OFF.
        * Moves the robot forward (`forword()`).
    * Else if the left IR sensor (`s3`) reads a value between 251 and 700 (likely indicating the line is to the left):
        * Turns the pump OFF.
        * Moves the robot backward for a short duration.
        * Turns the robot left for a short duration.
    * Else (if none of the above conditions are met):
        * Turns the pump OFF.
        * Stops the motors.
* Introduces a 10-millisecond delay at the end of the loop.

### Helper Functions

* **`servoPulse(int pin, int angle)`:**
    * Generates a PWM pulse for controlling the servo motor.
    * Calculates the pulse width in microseconds based on the input `angle` (a common formula for standard servo motors).
    * Sets the specified `pin` HIGH, delays for the calculated pulse width, sets the `pin` LOW, and then introduces a 50-millisecond delay.
* **`forword()`:** Sets the motor driver pins to move both motors forward.
* **`backword()`:** Sets the motor driver pins to move both motors backward.
* **`turnRight()`:** Sets the motor driver pins to make the robot turn right (by driving one motor backward and the other forward, or by stopping one and driving the other).
* **`turnLeft()`:** Sets the motor driver pins to make the robot turn left (by driving one motor forward and the other backward, or by driving one and stopping the other).
* **`Stop()`:** Sets the motor driver pins to stop both motors.

## Notes

* The threshold values for the IR sensors (e.g., 250, 350, 700, 800) will likely need to be adjusted based on the specific IR sensors being used, the surface being detected (the line), and the ambient light conditions. You can use the Serial Monitor to observe the sensor readings in different scenarios and fine-tune these values.
* The motor control logic in the line following section is basic. More sophisticated algorithms (like PID control) can be implemented for smoother and more accurate line following.
* The servo sweeping pattern and the activation of the pump are triggered by obstacle detection. The specific behavior (angles of sweep, duration of pump activation) can be modified in the `loop()` function.
* The `servoPulse()` function uses a fixed delay after setting the pulse low. This delay might need adjustment depending on the servo motor's specifications and desired speed.
* The turning logic (`turnRight()`, `turnLeft()`) assumes a differential drive robot configuration. The specific pin settings might need to be changed based on how your motors are connected to the motor driver.
* The pump control is a simple digital ON/OFF. More complex pump control (e.g., PWM for variable flow) could be implemented if needed.
