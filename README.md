# Self_Balancing_Robot
[youtube link: https://youtu.be/x-iZBCKg02A](https://youtu.be/x-iZBCKg02A)

<img width="803" alt="Screenshot 2025-06-21 at 12 10 08 AM" src="https://github.com/user-attachments/assets/d8daae9a-8ca8-499b-9b1e-d9d5e9366e6e" />

---
# Overview: 

ELEC 391 Electrical Engineering Design Studio. 

# Objective: 

To create a self-balancing bot that is able to:    
1) balance on the spot without moving past a ±4 cm mark,
2) go backwards and forwards 50 cm and pause,
3) turn 45 degrees and pause,
4) go up a ramp. 

# Hardware & Tools Used: 

- 3D‑printed bot body 
- 8 × 1.5 V rechargeable batteries 
- Arduino Nano 33 BLE Sense Rev2 with headers (ABX00070) 
- 2 × AS5600 magnetic encoders
- TCA9548A I²C multiplexer
- 2 × DRV8833 H‑bridge motor drivers (paralleled for more current) 
- Flutter (for Bluetooth controller app)    
Overall, we were able to balance within the ±2 cm range and successfully achieved all objectives. The robot was also able to climb a 10-degree ramp. 

# How the Robot Works: 

We programmed the Arduino to generate PWM signals that approximate a DC voltage, depending on the PWM value being written. These signals power two H-bridge drivers connected in parallel (for higher current output). This allows VCC to pass through, driving the motors. Different PWM signals are sent to AIN1/AIN2 (motor A) and BIN1/BIN2 (motor B), causing the wheels to turn in various directions at variable speeds. Control is achieved through these PWM signals from the microcontroller. 

The magnetic encoders were used to calculate wheel velocity. To determine the robot's tilt angle, we used the built-in gyroscope and accelerometer in the Arduino Nano. We followed the course documentation for the conversion formulas. 

# Balancing Logic: 

For basic balancing, we used the angle calculated from the two onboard sensors and compared it to the desired angle (0 degrees). The difference gave us the error. We used a PID controller to map this error angle into a PWM signal (ranging from 0 to 255). Since PWM must be a positive integer, I wrote helper functions to handle direction based on the PID output’s sign and speed based on its absolute value. If the PID output was positive, the wheels would go forward; if negative, they would reverse. 

# Velocity Control: 

After integrating the encoders and I2C multiplexer, we converted the encoder output into velocity. Then we compared actual velocity to a desired velocity using two different approaches: 

Version 1: We mapped the velocity difference directly to a PWM value and blended it with the original PID output using a weighted average (e.g., k * first_pwm + (1-k) * velocity_pwm). This allowed the robot to stay balanced while being able to move forward and backward. 

Version 2: We created a secondary PID loop that calculated a new desired angle based on the velocity error. This desired angle was then fed into the original angle PID. In this version, the robot would lean slightly in the desired direction to move forward/backward, but we would pause quickly after motion so that it didn’t keep accelerating and fall over. 

We chose Version 2 because it provided cleaner, smoother movement, especially after testing and fine-tuning different velocity and angle values.  

# Turning Logic: 

Turning required more nuanced control. We designed the robot to make one wheel (the left) spin slightly faster than the other to trace a curved path. The right wheel's speed was scaled by 0.99 to help offset any asymmetry. The left wheel was deliberately accelerated to initiate a turn, but this also introduced a swing-back issue as the balance system fought the induced imbalance. 

To reduce this effect, we manually adjusted how much forward and reverse motion was applied during turns: when turning, the robot would move slightly more forward than necessary, and when it started swinging back, it would reduce the reverse effort. This minimized the swing and helped the robot trace smoother arcs. 

To calibrate for physical asymmetry between the motors, we added correction factors like turn_L = 1 and turn_R = 1.1 (or -1.15 for reverse). These were tuned through trial and error to ensure forward and backward turns happened at roughly the same speed and radius. 

This logic isn't perfect, and it’s not based on an exact model, but it worked well enough for our goals. Here's a simplified version of our turning code: 

void forward(int num, int pwm){ //
  if(num == A){ //
    analogWrite(AIN1, 255); //
    analogWrite(AIN2, constrain(int((255 - pwm) * turn_L), 0, 255)); \\
  } else if(num == B){ \\
    analogWrite(BIN1, 255);    
    analogWrite(BIN2, constrain(int((255 - pwm) * turn_R), 0, 255));    
  }    
}    
  

The reverse logic follows a similar structure. 

# Testing Tools: 

We created a Flutter Bluetooth app with buttons that sent strings to the Arduino. Based on the strings received, we toggled between different angles, velocities, and other test parameters. This allowed our phones to serve as custom controllers, which was incredibly helpful during testing. 

We also implemented a keyboard test interface to adjust PID parameters over serial. This saved us time since we no longer had to reflash the code every time we changed a value. 

# Design Flow: 

We started by testing the Arduino’s built-in angle sensors. The accelerometer does not accumulate drift but is noisy, while the gyroscope is smooth but drifts over time. We graphed both readings live in Python and used a complementary filter to combine them. This also helped us determine the necessary sensor offset. 

Next, we tested the wheels by spinning them in different directions at various PWM levels and checking how their rotation correlated with the tilt angle. One wheel turned earlier and faster, so we reduced its PWM output slightly in software to match the other. 

Then, we built a Simulink model of the robot, estimating the center of mass, length, etc. We used auto-tuning in Simulink to get rough PID values and visualized the oscillations. These were used as a starting point for real-world tuning. 

For PID tuning, we increased Kp until we saw oscillation, then adjusted Ki to help it balance, and finally tweaked Kd to reduce oscillations. This process took many days and required constant trial and error. Anytime the robot hit something or we swapped the battery, we had to retune the system again. 

# Challenges: 

Our robot wasn’t perfectly balanced in terms of mass, so collisions or even small bumps would shift the center and require angle offset adjustments. The two wheels behaved slightly differently, so we had to experiment a lot to find a good compensation ratio. 

The night before the project was due, we returned to the lab to run a final test, and the robot suddenly stopped responding correctly. It began oscillating wildly and shot off to the side. After several hours of debugging, we found out that one of the wheel screws had loosened and the wheel was sliding on the shaft. This happened around 3 AM, and we had to essentially start over—rebalancing, re-tuning, and reconfiguring everything. 

# Moving Forward: 

Now that we know the system works, our next steps are to clean up and organize the code, and integrate timers for more autonomous control. Previously, most of our tests were done manually using toggle buttons. With everything validated, we can now encode sequences that run for specific durations. 

# Testing

https://github.com/user-attachments/assets/9a0a5298-2b6f-41c0-b1ad-c518d62518c2

# Tuning

https://github.com/user-attachments/assets/ab172b07-5f2b-4440-81bb-b1453649902a




