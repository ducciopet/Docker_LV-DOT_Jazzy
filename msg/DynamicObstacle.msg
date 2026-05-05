std_msgs/Header header
uint32 track_id

geometry_msgs/Pose pose       # position in map frame; orientation is identity (not estimated)
geometry_msgs/Vector3 size    # full extents: x_width, y_width, z_width in metres

geometry_msgs/Twist twist     # linear.x/y/z = Vx/Vy/Vz from Kalman; angular = 0
geometry_msgs/Accel accel     # linear.x/y/z = Ax/Ay/Az from Kalman; angular = 0

float32 confidence            # 1.0 = confirmed dynamic, 0.5 = confirmed static
bool is_static                # true when is_dynamic flag is false
bool is_yolo                  # true when detected by YOLO as a dynamic class
