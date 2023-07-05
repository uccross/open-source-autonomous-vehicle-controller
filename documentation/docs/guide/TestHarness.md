# Test Harness

![IMU Connections](../assets/images/TestHarness/IMU1.png)

![IMU Connections](../assets/images/TestHarness/IMU2.jpg)

for reference to the IMU documentation refer [here](https://learn.sparkfun.com/tutorials/sparkfun-9dof-imu-icm-20948-breakout-hookup-guide)

## IMU Test Harness

- File -> Open Project
![Open Project](../assets/images/first_project/openProject.png)

- Now navigate to -> open-source-autonomous-vehicle-controller -> lib -> Test_harness.X
you would be able to see the below window with the serial project open
<!-- ![Test harness](../assets/images/first_project/) -->

- Now Open Test_harness.X Project Properties (File -> Project Properties)
![Test_harness Project Properties](../assets/images/first_project/projectProperties.png)

- Choose Connected Hardware Tool to PICkit3

- Navigate to xc32-ld
![xc32-id](../assets/images/first_project/xc32id.png)

- Choose General as the option category
![General option category](../assets/images/first_project/general.png)

- set the Heap Size (bytes) to be 8000 bytes
![Heap Size](../assets/images/first_project/bytes.png)

- Click OK to save the modified Project Properties
![OK](../assets/images/first_project/ok.jpg)

- In the main.c file, set the IMU_test boolean as TRUE

- Click Clean and Build
![Clean and Build](../assets/images/first_project/CleanBuild.png)

- Click Make and Program Device
![Make and Program Device](../assets/images/first_project/MakeProgram.png)

- Open your preset serial terminal application (CoolTerm)

Output should be:
![Output for IMU](../assets/images/TestHarness/IMU3.png)

## Motor Test Harness

### Hardware Setup for Motor

![Motor Connections](../assets/images/TestHarness/MotorTest1.jpg)

![Motor Connections](../assets/images/TestHarness/MotorTest2.jpg)

![Motor Connections](../assets/images/TestHarness/MotorTest3.jpg)

### Software Setup for Motor

- In the main.c file, set the Servo_test boolean as TRUE or the Brushless_test boolean as TRUE depending on if you have a servo or brushless motor.

- Click Clean and Build
![Clean and Build](../assets/images/first_project/CleanBuild.png)

- Click Make and Program Device
![Make and Program Device](../assets/images/first_project/MakeProgram.png)

- Open your preset serial terminal application (CoolTerm)

Output should be:

## GPS Test Harness

### Hardware Setup for GPS

![GPS Test Harness](../assets/images/TestHarness/GPS1.jpg)

### Software Setup for GPS

- In the main.c file, set the GPS_test boolean as TRUE.

- Click Clean and Build
![Clean and Build](../assets/images/first_project/CleanBuild.png)

- Click Make and Program Device
![Make and Program Device](../assets/images/first_project/MakeProgram.png)

- Open your preset serial terminal application (CoolTerm)

Output should be:
![GPS Test Output](../assets/images/TestHarness/GPS2.png)

## RC Receiver Test Harness

### Hardware Setup for RC Receiver

![RC Connections](../assets/images/TestHarness/RC1.png)

![RC Connections](../assets/images/TestHarness/RC2.jpg)

### Software Setup for RC Receiver

- In the main.c file, set the Radio_test boolean as TRUE.

- Click Clean and Build
![Clean and Build](../assets/images/first_project/CleanBuild.png)

- Click Make and Program Device
![Make and Program Device](../assets/images/first_project/MakeProgram.png)

- Open your preset serial terminal application (CoolTerm)

Output should be:

![RC Receiver test output](../assets/images/TestHarness/RC3.png)

## Mavlink Test Harness

### Hardware Setup for Mavlink

![Mavlink Test Harness](../assets/images//TestHarness/MavLink1.png)

![Mavlink Test Harness](../assets/images//TestHarness/MavLink2.jpg)

![Mavlink Test Harness](../assets/images//TestHarness/MavLink3.jpg)

### Software Setup for RC Mavlink

- Open QGroundControl

![QGroundControl](../assets/images//TestHarness/Mavlink4.png)

- In the main.c file, set the Heartbeat_test boolean as TRUE.

- Click Clean and Build
![Clean and Build](../assets/images/first_project/CleanBuild.png)

- Click Make and Program Device
![Make and Program Device](../assets/images/first_project/MakeProgram.png)

- In QGroundControl, make sure that the upper left hand corner says “Armed”

- Click on the Q in the upper left hand corner for the menu to appear

![QGroundControl](../assets/images//TestHarness/Mavlink5.png)

- Click on Analyze Tools -> MAVlink Inspector

![QGroundControl](../assets/images//TestHarness/Mavlink6.png)

- Make sure that you’re receiving a heartbeat
