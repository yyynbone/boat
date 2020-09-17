#cmd type
int8 IMU_FITERR = -2   #get imu Geomagnetic fiterr
int8 LIGHT = -1        #data_int[0]:control_flag, data_int[1]:left light status, data_int[2]:right light status
int8 STOP = 0          #motor stop
int8 SPEED = 1         #data_int[0] = linear_velocity, data_int[1]=angular_velocity
                       #data_float32[0] = linear_velocity, data_float32[1]=angular_velocity
int8 SPEED_SINGLE = 2  #data_int[0] = left_motor_power, data_int[1]=right_motor_power
                       #data_float32[0] = left_motor_power, data_float32[1]=right_motor_power

int8 cmd

int8[] data_int
float32[] data_float32

