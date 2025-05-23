# Task 1 – Data PloTng [1 mark]
In order to check your results, validate your design choices, and debug your design, you will
need a way to plot your sensor readings in a graphical manner. Your task 1 is to figure out a
way to plot data in real Ame from an Arduino system (in parAcular the Nano 33 BLE sense) on
a PC or Mac screen. As a real-Ame sensor data source, you can use any of the sensors built-
into the Arduino board, or random readings from any of the analog input pins. The built-in
Serial PloMer of the Arduino IDE is not an acceptable soluAon. You should not use any external
circuitry (resistors, potenAometers, etc.)
Hint: Python is one of the most useful languages for data manipulaAon and plodng. If you do
not know Python, it is fine to use a LLM such as ChatGPT and direct it to write your code. If
you use LLMs, submit your prompt(s) with this assignment, not only your code.
Grading: demonstrate your real-Ame data plot to the TA, answer TA’s quesAons.
Deliverables:
- task1.ino (Arduino code)
- task1.py (Python code)
# Task 2 – CompuKng angles with accelerometer readings [1 mark]
Write Arduino code to collect accelerometer data from the BMI270_BMM150 IMU and
compute the Alt angle of the sensor. Display this angle value using the plodng method you
developed for task 1. Angles must be calculated exactly as indicated earlier in this handout,
no extra processing (i.e. filtering) is allowed.
Grading: demonstrate your Alt angle calculaAon and plodng to the TA, answer TA’s
quesAons.
# Task 3 – CompuKng angles with gyro readings [1 mark]
IdenAcal to task 3, but use gyroscope readings and calculaAons.
Grading: demonstrate your Alt angle calculaAon and plodng to the TA, answer TA’s
quesAons.
# Task 4 – Calculate angles using a complementary filter [2 marks]
Using the complementary filter method, compute the Alt angle. You will have to experiment
and tune the value of the K coefficient to obtain a stable and accurate reading. On the same
plot, show the three datasets corresponding to the three methods, simultaneously.
Grading: demonstrate your complementary filter. Explain your tuning strategy for the k
coefficient to the TA.
For all tasks, demonstrate your angle readings at three different angles: 0° (verAcal), ±15°,
and ±30°.
