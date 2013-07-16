Data.timestamp = 0;

Data.state = nav_msgs.Odometry;
Data.state.pose.pose.orientation.w = 1.0;

Data.imu = sensor_msgs.Imu;
Data.imu.orientation.w = 1.0;

Data.fmp = fmp_msgs.SensorData;

Data.global = sensor_msgs.NavSatFix;
Data.fix.status.status = -1;

Data.fix_velocity = geometry_msgs.Vector3Stamped;

Data.magnetic = geometry_msgs.Vector3Stamped;

Data.supply = hector_uav_msgs.Supply;
Data.status = rosgraph_msgs.Log;

Data.euler = nan(1,3);
Data.course = nan;
Data.compassheading = nan;

Data.v_mittel = nan;
Data.start = 0;
Data.flugzeit = 0;
Data.landung = 0;

initialized = 1;
