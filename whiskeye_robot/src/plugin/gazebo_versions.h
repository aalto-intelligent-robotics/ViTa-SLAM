
////    changes at version 7

#if GAZEBO_MAJOR_VERSION < 7

#define GAZEBO_SENSOR_TYPE GetType
#define GAZEBO_SENSOR_UPDATE_RATE GetUpdateRate
#define GAZEBO_CAMERA_SENSOR_IMAGE_WIDTH GetImageWidth
#define GAZEBO_CAMERA_SENSOR_IMAGE_HEIGHT GetImageHeight
#define GAZEBO_CAMERA_SENSOR_IMAGE_DATA GetImageData
#define GAZEBO_IMU_SENSOR_LINEAR_ACCELERATION GetLinearAcceleration
#define GAZEBO_SONAR_SENSOR_RANGE GetRange
#define GAZEBO_SCOPED_NAME GetScopedName

#else

#define GAZEBO_SENSOR_TYPE Type
#define GAZEBO_SENSOR_UPDATE_RATE UpdateRate
#define GAZEBO_CAMERA_SENSOR_IMAGE_WIDTH ImageWidth
#define GAZEBO_CAMERA_SENSOR_IMAGE_HEIGHT ImageHeight
#define GAZEBO_CAMERA_SENSOR_IMAGE_DATA ImageData
#define GAZEBO_IMU_SENSOR_LINEAR_ACCELERATION LinearAcceleration
#define GAZEBO_SONAR_SENSOR_RANGE Range
#define GAZEBO_SCOPED_NAME ScopedName

#endif



////    changes at version 8

#if GAZEBO_MAJOR_VERSION < 8

#define GAZEBO_GET_JOINT_POS(x) GetAngle(x).Radian()
#define VECTOR3 math::Vector3
#define QUATERNION math::Quaternion
#define GAZEBO_WORLD_POSE GetWorldPose
#define WORLD_POSE math::Pose
#define GAZEBO_GET_SIM_TIME GetSimTime
#define __POSE_POS(pose) ((pose).pos)
#define __POSE_ROT(pose) ((pose).rot)
#define __POSE_X(pose) ((pose).pos.x)
#define __POSE_Y(pose) ((pose).pos.y)
#define __POSE_Z(pose) ((pose).pos.z)
#define __POSE_THETA(pose) ((pose).rot.GetYaw())
#define __VECTOR3_X(v) ((v).x)
#define __VECTOR3_Y(v) ((v).y)
#define __VECTOR3_Z(v) ((v).z)
#define __POSE_GET_INVERSE(pose) ((pose).GetInverse())
#define __VECTOR_LENGTH(v) ((v).GetLength())

#else

#define GAZEBO_GET_JOINT_POS(x) Position(x)
#define VECTOR3 math::Vector3d
#define QUATERNION math::Quaternion<double>
#define GAZEBO_WORLD_POSE WorldPose
#define WORLD_POSE ignition::math::Pose3d
#define GAZEBO_GET_SIM_TIME SimTime
#define __POSE_POS(pose) ((pose).Pos())
#define __POSE_ROT(pose) ((pose).Rot())
#define __POSE_X(pose) ((pose).Pos().X())
#define __POSE_Y(pose) ((pose).Pos().Y())
#define __POSE_Z(pose) ((pose).Pos().Z())
#define __POSE_THETA(pose) ((pose).Rot().Yaw())
#define __VECTOR3_X(v) ((v).X())
#define __VECTOR3_Y(v) ((v).Y())
#define __VECTOR3_Z(v) ((v).Z())
#define __POSE_GET_INVERSE(pose) ((pose).Inverse())
#define __VECTOR_LENGTH(v) ((v).Length())

#endif


