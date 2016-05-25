
time Stamp

float32[3] AbsOrientation # (Euler Vector, 100Hz)
                               # Three axis orientation data based on a 360° sphere
#geometry_msgs/Quaternion AbsOrientationQ # (Quaterion, 100Hz)
                               # Four point quaternion output for more accurate data manipulation
float32[3] AngularVelocity     # (100Hz)
                               # Three axis of 'rotation speed' in rad/s
float32[3] AccelerationVector  # (100Hz)
                               # Three axis of acceleration (gravity + linear motion) in m/s^2
float32[3] MagneticField       # (20Hz)
                               # Three axis of magnetic field sensing in micro Tesla (uT)
float32[3] LinearAcceleration  # (100Hz)
                               # Three axis of linear acceleration data (acceleration minus gravity) in m/s^2
float32[3] GravityVector       # (100Hz)
                               # Three axis of gravitational acceleration (minus any movement) in m/s^2
float32    Temperature         # (1Hz)
                               # Ambient temperature in degrees celsius
