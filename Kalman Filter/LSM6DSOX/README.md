I took the MPU6050 kalman filter code from https://github.com/ProFL/KalmanMPU6050/blob/master/examples/SerialPlotter/SerialPlotter.ino aand adapted it for use on the LSM6DSOX 6DOF IMU. Here are some things to note to adapt the code to other IMUs:

# Arduino sketch does not need edits other than switching header file
In my .ino file, I call the library "KalmanMPU6050.h" but I have modified it to work with the LSM. I didnt take time to modfy it part because its too much work atm, and part because I dont want to break the code before I push to github.

# Communication between device and peripheral is handled by cpp file
This depends on the IMU used. I have a comment in the cpp file on how the LSM is different from MPU6050 so that anyone can make similar changes by looking at IMU datasheet/chatGPT. There are enoough comments in the .cpp file imo. 
***I'm sorry about the confusion but even if you see functions with MPU in their name, they have been modified for use with the LSM, SO DON'T GET CONFUSED, And if you can, rename the functions if I have not already done so.*** 
For wiring diagram, follow https://learn.adafruit.com/lsm6dsox-and-ism330dhc-6-dof-imu/arduino. I used I2C.

# Moving average filter to account for bias
I noticed that just implementing the kalman filter made the steady state be not at zero. To tackle this, I take the last 10 readings, average them, then subtract them to get steady state value (foor roll and pitch individually). 
So far, the error has stayed within ~0.4% after 30 minutes of runtime for roll and pitch.

#### this readme has been written by Mustahsin Zarif, so you can ask me questions about the code. I'll answer if I remember what I did.
