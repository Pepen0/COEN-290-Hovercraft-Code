# COEN-290-Hovercraft-Code
The official code for our design that got us to second place 

The hovercraft's latest control algorithm presented in this report is designed to navigate the hovercraft through its environment by making use of various sensors and adjusting the angles of its fans. The algorithm can be divided into four main sections: getting started, preparing the system, the core loop, and auxiliary functions.

In the first section, the necessary libraries are imported and constants and global variables are declared. The variables include sensor addresses, pin assignments, initial configurations, and sensor measurements. The MPU-6050 sensor, a combination of an accelerometer and gyroscope, is initialized with the appropriate I2C address and settings. The servo and fans are set up with their designated pins and starting configurations. Lastly, the ultrasonic sensors are initialized with their respective pins and maximum distance parameters.

The second section focuses on preparing the system for operation. I2C communication is established with the MPU-6050 sensor and activated. Serial communication is initiated at a 9600 baud rate. The servo motor is connected to the appropriate pin, and the Z-axis gyroscope bias is computed. Fan control pins are assigned as outputs, and their initial speeds are established. The initial time is recorded in the 'tempo' variable.

The core loop of the algorithm serves as the heart of the hovercraft's control. It computes the current yaw using gyroscope readings and obtains distances from the two ultrasonic sensors. Based on these measurements, the hovercraft decides whether to turn left, turn right, or proceed straight. The servo angle is adjusted as needed, and fan speeds are determined according to the hovercraft's movements. The servo angle is then restricted within a specific range, and the servo motor and fans are updated with the new angle and fan speeds.

Lastly, the algorithm employs auxiliary functions to enhance its performance. The 'calculate_gz_bias()' function calculates the Z-axis gyroscope's bias/error to improve the precision of the yaw calculation. The 'getDistance()' function returns the distance in 
centimeters as measured by the ultrasonic sensors, using the NewPing library. Overall, the hovercraft's motion is managed by adjusting the fans' orientation based on readings from the MPU-6050 sensor and ultrasonic sensors.
