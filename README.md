# This is a GNSS/IMU loosly coupling MATLAB program.

### GNSS format:
GPSTime, rtk_state, Lat, Lon, Height, Vel_x, Vel_y, Vel_z.

### IMU format:
GPSTime, Gyro_x, Gyro_y, Gyro_z, Acc_x, Acc_y, Acc_z.

## Build and Run

### Input
GNSS frequency, IMU frequency,IMU bias in Kalamn filter Q matrix.

### Select
You can choose kinds of update methords:
POSITION_FILTER_UPDATE, XYZVEL_FILTER_UPDATE, NEDVEL_FILTER_UPDATE, and POSVEL_FILTER_UPDATE.

## Note

Still in update, thank you for your suggestions! iXR forever!
