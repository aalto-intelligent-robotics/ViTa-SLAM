
#include "libwhiskeye_gazebo.h"
#include "whisker_pose.h"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/sensors/SensorManager.hh>

#include <ignition/math.hh>
using namespace ignition;

#include "gazebo_versions.h"

#include <stdio.h>
#include <string>
using namespace std;

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <whiskeye_msgs/bridge_u.h>
#include <image_transport/image_transport.h>

#define MAX_SENSOR_COUNT 32
#define NUM_50HZ_SENSORS 25

#define __COUT_W cout << "**** WARNING **** "
#define __ERROR(msg) do { cout << "\n**** ERROR **** " << msg << "\n" << endl; throw(msg); } while(false)
#define __UNUSED(param) do { (void)param; } while(false)

struct XY
{
	float x;
	float y;
};

struct WhiskerXY
{
	struct XY xy[ROBOT_FS_SCALE][ROBOT_ROW_COUNT][ROBOT_COL_COUNT];
};



namespace gazebo
{
	struct JointDesc
	{
		string name;
		physics::JointPtr joint;
	};

	struct JointPose
	{
		double o[3]; // joint centre (origin)
		double a[3]; // 100mm off in joint axis direction (+ve y in whisker canonical model)
		double f[3]; // 100mm off in "forward direction" (towards whisker tip, +ve z in whisker canonical model)
	};

	WORLD_POSE transform_zero_y(WORLD_POSE T, WORLD_POSE q)
	{
		WORLD_POSE Tq = q * T;
		double f = atan2(__POSE_Y(Tq), __POSE_X(Tq));
		T = T * WORLD_POSE(0, 0, 0, 0, 0, -f);
		return T;
	}

	WORLD_POSE transform_zero_x(WORLD_POSE T, WORLD_POSE q)
	{
		WORLD_POSE Tq = q * T;
		double f = atan2(__POSE_X(Tq), __POSE_Z(Tq));
		T = T * WORLD_POSE(0, 0, 0, 0, -f, 0);
		return T;
	}

	WORLD_POSE transform_zero_x2(WORLD_POSE T, WORLD_POSE q)
	{
		WORLD_POSE Tq = q * T;
		double f = atan2(__POSE_X(Tq), __POSE_Y(Tq));
		T = T * WORLD_POSE(0, 0, 0, 0, 0, f);
		return T;
	}

#define __NORM(x) ((x).pos.GetLength())

	WORLD_POSE get_pose_transform(const JointPose* wpose)
	{
		/*
			This version is appropriate for whisker canonical frames
		*/

		//	construct objects we can use for numerical search
		WORLD_POSE o(wpose->o[0], wpose->o[1], wpose->o[2], 0, 0, 0);
		WORLD_POSE y(wpose->a[0], wpose->a[1], wpose->a[2], 0, 0, 0);
		WORLD_POSE z(wpose->f[0], wpose->f[1], wpose->f[2], 0, 0, 0);

		//	initial guess puts o at origin
		WORLD_POSE T = __POSE_GET_INVERSE(o);

		/*
		cout << "----------------" << endl;
		cout << (o * T).pos << endl;
		cout << (y * T).pos << " (" << __NORM(y * T) << ")" << endl;
		cout << (z * T).pos << " (" << __NORM(z * T) << ")" << endl;
		*/

		//	zero y coordinate of z
		T = transform_zero_y(T, z);

		/*
		cout << "----------------" << endl;
		cout << (o * T).pos << endl;
		cout << (y * T).pos << " (" << __NORM(y * T) << ")" << endl;
		cout << (z * T).pos << " (" << __NORM(z * T) << ")" << endl;
		*/

		//	zero x coordinate of z
		T = transform_zero_x(T, z);

		/*
		cout << "----------------" << endl;
		cout << (o * T).pos << endl;
		cout << (y * T).pos << " (" << __NORM(y * T) << ")" << endl;
		cout << (z * T).pos << " (" << __NORM(z * T) << ")" << endl;
		*/

		//	zero x coordinate of y
		T = transform_zero_x2(T, y);

		/*
		cout << "----------------" << endl;
		cout << (o * T).pos << endl;
		cout << (y * T).pos << " (" << __NORM(y * T) << ")" << endl;
		cout << (z * T).pos << " (" << __NORM(z * T) << ")" << endl;
		*/

		//	check
		if (__VECTOR_LENGTH(__POSE_POS(o * T) - VECTOR3(0, 0, 0)) > 0.001)
			__ERROR("bad O");
		if (__VECTOR_LENGTH(__POSE_POS(y * T) - VECTOR3(0, 0.1, 0)) > 0.001)
			__ERROR("bad Y");
		if (__VECTOR_LENGTH(__POSE_POS(z * T) - VECTOR3(0, 0, 0.1)) > 0.001)
			__ERROR("bad Z");

		//	ok
		return T;
	}

	WORLD_POSE get_pose_transform_2(const JointPose* wpose)
	{
		/*
			This version is appropriate for the HEAD canonical frame
		*/

		//	construct objects we can use for numerical search
		WORLD_POSE o(wpose->o[0], wpose->o[1], wpose->o[2], 0, 0, 0);
		WORLD_POSE z(wpose->a[0], wpose->a[1], wpose->a[2], 0, 0, 0);
		WORLD_POSE x(wpose->f[0], wpose->f[1], wpose->f[2], 0, 0, 0);

		WORLD_POSE T;

#ifdef SHOW_GET_POSE_TRANSFORM_2
		cout << "----------------" << endl;
		cout << (o * T).pos << endl;
		cout << (z * T).pos << " (" << __NORM(z * T) << ")" << endl;
		cout << (x * T).pos << " (" << __NORM(x * T) << ")" << endl;
#endif // SHOW_GET_POSE_TRANSFORM_2

		//	initial guess puts o at origin
		T = __POSE_GET_INVERSE(o);

#ifdef SHOW_GET_POSE_TRANSFORM_2
		cout << "----------------" << endl;
		cout << (o * T).pos << endl;
		cout << (z * T).pos << " (" << __NORM(z * T) << ")" << endl;
		cout << (x * T).pos << " (" << __NORM(x * T) << ")" << endl;
#endif // SHOW_GET_POSE_TRANSFORM_2

		//	zero y coordinate of z
		T = transform_zero_y(T, z);

#ifdef SHOW_GET_POSE_TRANSFORM_2
		cout << "----------------" << endl;
		cout << (o * T).pos << endl;
		cout << (z * T).pos << " (" << __NORM(z * T) << ")" << endl;
		cout << (x * T).pos << " (" << __NORM(x * T) << ")" << endl;
#endif // SHOW_GET_POSE_TRANSFORM_2

		//	zero x coordinate of z
		T = transform_zero_x(T, z);

#ifdef SHOW_GET_POSE_TRANSFORM_2
		cout << "----------------" << endl;
		cout << (o * T).pos << endl;
		cout << (z * T).pos << " (" << __NORM(z * T) << ")" << endl;
		cout << (x * T).pos << " (" << __NORM(x * T) << ")" << endl;
#endif // SHOW_GET_POSE_TRANSFORM_2

		//	zero y coordinate of x
		T = transform_zero_y(T, x);

#ifdef SHOW_GET_POSE_TRANSFORM_2
		cout << "----------------" << endl;
		cout << (o * T).pos << endl;
		cout << (z * T).pos << " (" << __NORM(z * T) << ")" << endl;
		cout << (x * T).pos << " (" << __NORM(x * T) << ")" << endl;
#endif // SHOW_GET_POSE_TRANSFORM_2

		//	check
		if (__VECTOR_LENGTH(__POSE_POS(o * T) - VECTOR3(0, 0, 0)) > 0.001)
			__ERROR("bad O");
		if (__VECTOR_LENGTH(__POSE_POS(z * T) - VECTOR3(0, 0, 0.1)) > 0.001)
			__ERROR("bad Z");
		if (__VECTOR_LENGTH(__POSE_POS(x * T) - VECTOR3(0.1, 0, 0)) > 0.001)
			__ERROR("bad X");

		//	ok
		return T;
	}

	struct Whiskeye_ModelPlugin : public ModelPlugin
	{
		static void ros_init()
		{
			//	initialize layer if not already
			if (!ros::isInitialized())
			{
				cout << "initializing ROS..." << endl;
				int argc = 0;
				char **argv = NULL;
				ros::init(argc, argv, "whiskeye", ros::init_options::NoSigintHandler);

				//	use check() to ensure that we can contact the ROS master,
				//	otherwise we will hang when we create our first NodeHandle
				int count = 10;
				while (count)
				{
					cout << "attempting to contact ROS master..." << endl;
					if (ros::master::check())
						return;

					usleep(1000000);
					count--;
				}

				//	failed to contact master
				__ERROR("could not contact ROS master");
			}
		}

		Whiskeye_ModelPlugin()
			:
			ModelPlugin()
		{
			cout << "Whiskeye_ModelPlugin() [ctor]" << endl;
			ros_init();

			//	create ROS handle
			output.h_ros = new ros::NodeHandle;
		}

		~Whiskeye_ModelPlugin()
		{
			cout << "~Whiskeye_ModelPlugin() [dtor]" << endl;
			delete output.h_ros;
		}

		physics::LinkPtr GetLink(string name)
		{
			physics::LinkPtr ret = state.model->GetLink(name);
			if (ret)
			{
				//	return it
				return ret;
			}

			//	report existing links
			physics::Link_V links = state.model->GetLinks();
			for (uint32_t i=0; i<links.size(); i++)
			{
				physics::LinkPtr link = links[i];
				cout << "link: " << link->GetName() << endl;
			}

			//	fail
			__ERROR("could not find link \"" + name + "\"");
		}

		void connect(event::ConnectionPtr connection)
		{
			if (state.sensors_connected == MAX_SENSOR_COUNT)
				__ERROR("ran out of space to store sensor connections");
			state.connection[state.sensors_connected++] = connection;
		}

		void SetPositionPID(string joint_name, double scale, double dscale)
		{
			double P = 10.0 * scale;
			double I = 0.0 * scale;
			double D = 10.0 * dscale * scale;
			double imax = 0.0 * scale;
			double imin = -imax;
			double cmdMax = 5.0 * scale;
			double cmdMin = -cmdMax;
			common::PID pid(P, I, D, imax, imin, cmdMax, cmdMin);
			state.controller->SetPositionPID(joint_name, pid);
			
			/*
			THIS DOES NOT WORK
			
			//	clear any associated velocity controller
			common::PID pid_default(0.0, 0.0, 0.0);
			state.controller->SetVelocityPID(joint_name, pid_default);
			*/
		}

		void SetVelocityPID(string joint_name, double scale, double iscale, double dscale)
		{
			double P = 10.0 * scale;
			double I = 10.0 * iscale * scale;
			double D = 10.0 * dscale * scale;
			double imax = 10.0 * scale;
			double imin = -imax;
			double cmdMax = 5.0 * scale;
			double cmdMin = -cmdMax;
			common::PID pid(P, I, D); //P, I, D, imax, imin, cmdMax, cmdMin);
			state.controller->SetVelocityPID(joint_name, pid);
			
			/*
			THIS DOES NOT WORK
			
			//	clear any associated position controller
			common::PID pid_default(0.0, 0.0, 0.0);
			state.controller->SetPositionPID(joint_name, pid_default);
			*/
		}

		void SetPosePIDs(bool using_cmd_pos)
		{
			//	if already set
			if (state.have_selected_using_cmd_pos)
			{
				//	warn if user is using wrong one - for some reason, if we try
				//	to switch in Gazebo, some trace of the old one is left, and
				//	I can't figure out how to remove it. so we just only allow
				//	use of one per session, which should be fine
				if (using_cmd_pos != state.using_cmd_pos)
					__COUT_W << "cannot mix cmd_pos and cmd_vel in one session" << endl;
				
				//	ok
				return;
			}
		
			//	set
			state.using_cmd_pos = using_cmd_pos;
			state.have_selected_using_cmd_pos = true;

			//	if changed
			if (using_cmd_pos)
			{
				SetPositionPID(state.model->GetName() + "::pose_x", ROBOT_POSE_POS_PID_P_0, ROBOT_POSE_POS_PID_D);
				SetPositionPID(state.model->GetName() + "::pose_y", ROBOT_POSE_POS_PID_P_1, ROBOT_POSE_POS_PID_D);
				SetPositionPID(state.model->GetName() + "::pose_theta", ROBOT_POSE_POS_PID_P_2, ROBOT_POSE_POS_PID_D);
			}
			else
			{
				SetVelocityPID(state.model->GetName() + "::pose_x", ROBOT_POSE_VEL_PID_P_0, ROBOT_POSE_VEL_PID_I, ROBOT_POSE_VEL_PID_D);
				SetVelocityPID(state.model->GetName() + "::pose_y", ROBOT_POSE_VEL_PID_P_1, ROBOT_POSE_VEL_PID_I, ROBOT_POSE_VEL_PID_D);
				SetVelocityPID(state.model->GetName() + "::pose_theta", ROBOT_POSE_VEL_PID_P_2, ROBOT_POSE_VEL_PID_I, ROBOT_POSE_VEL_PID_D);
			}
		}
		
		JointDesc GetJoint(string joint_name)
		{
			//	get joints (from joint controller)
			std::map< std::string, physics::JointPtr > joints =
				state.controller->GetJoints();

			//	search
			joint_name = state.model->GetName() + "::" + joint_name;
			for(std::map< std::string, physics::JointPtr >::iterator it = joints.begin();
						it != joints.end(); ++it)
			{
				if (it->first == joint_name)
				{
					//	return it
					JointDesc desc;
					desc.name = joint_name;
					desc.joint = it->second;
					return desc;
				}
			}

			//	fail
			for(std::map< std::string, physics::JointPtr >::iterator it = joints.begin();
						it != joints.end(); ++it)
			{
				cout << it->first << endl;
			}
			__ERROR("joint \"" + joint_name + "\" not found (see above for list)");
		}

		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
		{
			cout << "Load()" << endl;

			//	store pointer to model
			state.model = _parent;

			//	report node name
			cout << "model instance name: " << state.model->GetName() << endl;

			//	get links
			state.link_body = GetLink("body");
			cout << "link_body: " << state.link_body << " (" << state.link_body->GetName() << ")" << endl;
			state.link_head = GetLink("head");
			cout << "link_head: " << state.link_head << " (" << state.link_head->GetName() << ")" << endl;
			for (int r=0; r<ROBOT_ROW_COUNT; r++)
			{
				for (int c=0; c<ROBOT_COL_COUNT; c++)
				{
					stringstream ss;
					ss << "whisker" << (r+1) << "_" << (c+1);
					state.link_whisker[r][c] = GetLink(ss.str());
				}
			}

////	SENSORS

			//	sensor summary
			sensors::Sensor_V ss = sensors::SensorManager::Instance()->GetSensors();
			cout << "SensorManager: " << ss.size() << " sensors are " <<
				(sensors::SensorManager::Instance()->SensorsInitialized()
					? "initialized" : "**** not initialized ****") << endl;
			/*
			for (int i=0; i<ss.size(); i++)
			{
				sensors::SensorPtr s = ss[i];
				cout << "sensor: " << s->Name() << endl;
			}
			*/

#define __GET_SENSOR(id, name, callback) do { \
	sens.id = sensors::SensorManager::Instance()->GetSensor(name); \
	if (!sens.id) __ERROR(("sensor not found \"" + string(name) + "\"")); \
	connect(sens.id->ConnectUpdated(boost::bind(&Whiskeye_ModelPlugin::callback, this, sens.id))); \
} while(false)

			//	get sensors
			__GET_SENSOR(imu_body, "imu_body", OnIMUUpdate);
			__GET_SENSOR(cam[0], "cam0", OnCamUpdate);
			__GET_SENSOR(cam[1], "cam1", OnCamUpdate);
			__GET_SENSOR(cam[2], "cam2", OnCamUpdate);

			//	get contact sensors
			for (int r=0; r<ROBOT_ROW_COUNT; r++)
			{
				for (int c=0; c<ROBOT_COL_COUNT; c++)
				{
					stringstream ss;
					ss << "whisker" << (r+1) << "_" << (c+1) << "_contact";
					__GET_SENSOR(contact[r][c], ss.str().c_str(), OnContactUpdate);
				}
			}

////	JOINTS

			//	get joint controller
			state.controller = state.model->GetJointController();
			if (!state.controller)
				__ERROR("failed to get joint controller");

			//	find pose joints
			state.joints_pose[0] = GetJoint("pose_x");
			state.joints_pose[1] = GetJoint("pose_y");
			state.joints_pose[2] = GetJoint("pose_theta");

			//	find neck joints
			state.joints_neck[0] = GetJoint("body_neck");
			state.joints_neck[1] = GetJoint("neck_gmbl");
			state.joints_neck[2] = GetJoint("gmbl_head");
			
			//	initialise their controllers
			SetPositionPID(state.joints_neck[0].name, ROBOT_NECK_PID_P_0, ROBOT_NECK_PID_D);
			SetPositionPID(state.joints_neck[1].name, ROBOT_NECK_PID_P_1, ROBOT_NECK_PID_D);
			SetPositionPID(state.joints_neck[2].name, ROBOT_NECK_PID_P_2, ROBOT_NECK_PID_D);
			
			//	find whisker joints
			for (int row=0; row<ROBOT_ROW_COUNT; row++)
			{
				for (int col=0; col<ROBOT_COL_COUNT; col++)
				{
					stringstream ss;
					ss << "head_whisker" << (row+1) << "_" << (col+1);
					state.joints_whisker[row][col] = GetJoint(ss.str());
					SetPositionPID(state.joints_whisker[row][col].name, ROBOT_WHISKER_PID_P, ROBOT_WHISKER_PID_D);
				}
			}

////	WHISKER MAPPING

			/*
				We need the initial state of the whiskers (their "world pose") because we'll
				use this to map contacts in their run-time FOR back to the initial whisker
				FOR for interpretation. To discover this, we set the whole model as "static"
				in the SDF, to make sure we get this code to run before anything moves, then
				we measure these poses here. Thankfully, they are all [0 0 0 0 0 0] (because
				that is how the SDF model is put together) so after discovering this, we can
				retire this code, safe in that knowledge.

				Initial world pose of all whiskers: [0 0 0 0 0 0]
			*/

			/*
			//	for each whisker
			for (int r=0; r<ROBOT_ROW_COUNT; r++)
			{
				for (int c=0; c<ROBOT_COL_COUNT; c++)
				{
					//	get link
					stringstream ss;
					ss << "whisker" << (r+1) << "_" << (c+1);
					physics::LinkPtr link = GetLink(ss.str());
					if (!link)
						__ERROR("whisker link not found");

					//	get pose
					const WORLD_POSE& pose = link->GAZEBO_WORLD_POSE();
					cout << ss.str() << " : " << pose << endl;
				}
			}
			*/

			/*
				Having mapped contacts into the initial FOR for each whisker, we then need
				to map them into some canonical FOR. We choose an FOR pointing directly up
				in +z, since it's intuitive and also since then +x and +y map directly to
				the x/y outputs we ultimately need to generate. This mapping is implicit in
				the STLs that we start with, since each whisker's pose at start-up is encoded
				in its actual position there. We could in principle measure it here, but it's
				somewhat easier to measure it during model creation and pass it in here using
				a header file. In fact, since the API available here is good at handling poses
				and the external file has access to the rotaxe objects, we'll get the model
				creation process to recover three reference points (joint centre, joint axis,
				whisker tip) and then use the API available here to convert that to a pose
				that will map each whisker's physical FOR onto the canonical whisker FOR.
			*/

			//	read whisker_pose
			if (sizeof(whisker_pose) != (ROBOT_WHISKER_COUNT * 9 * sizeof(double)))
				__ERROR("bad whisker_pose array size");
			const JointPose* joint_pose = (const JointPose*) whisker_pose;
			for (int r=0; r<ROBOT_ROW_COUNT; r++)
			{
				for (int c=0; c<ROBOT_COL_COUNT; c++)
				{
					//	find transform that brings this initial whisker
					//	pose back to the canonical pose
					WORLD_POSE T = get_pose_transform(joint_pose);

					//	store
					state.inverse_initial_whisker_pose[r][c] = T;

					//	advance
					joint_pose++;
				}
			}

			//	read neck_pose
			if (sizeof(neck_pose) != (3 * 9 * sizeof(double)))
				__ERROR("bad neck_pose array size");
			joint_pose = (const JointPose*) neck_pose;
			for (int i=0; i<3; i++)
			{
				//	find transform that brings this initial whisker
				//	pose back to the canonical pose
				WORLD_POSE T = get_pose_transform_2(joint_pose);

				//	store
				state.inverse_initial_neck_pose[i] = T;

				//	advance
				joint_pose++;
			}

////	INTERFACE

/*
			//	function not currently used
			//	connect to physics update event (1ms, typically)
			connect(event::Events::ConnectWorldUpdateBegin(
					boost::bind(&Whiskeye_ModelPlugin::OnWorldUpdate, this, _1)
					));
*/

			//	publish
			string topic_root = "/whiskeye";
			image_transport::ImageTransport it(*output.h_ros);
#define __ADVERTISE(field, type, name) \
	output.field.pub = output.h_ros->advertise<type> \
		((topic_root + name).c_str(), ROS_SEND_QUEUE_SIZE);
#define __IT_ADVERTISE(field, name) \
	output.field.pub = it.advertise \
		((topic_root + name).c_str(), ROS_SEND_QUEUE_SIZE);
			__ADVERTISE(bumper, std_msgs::Bool, "/body/bumper");
			__ADVERTISE(imu_body, sensor_msgs::Imu, "/body/imu_body");
			__ADVERTISE(pose, geometry_msgs::Pose2D, "/body/pose");
			__ADVERTISE(bridge_u, whiskeye_msgs::bridge_u, "/head/bridge_u");
			__ADVERTISE(xy, std_msgs::Float32MultiArray, "/head/xy");
			__ADVERTISE(contact_head, std_msgs::Float32MultiArray, "/head/contact_head");
			__ADVERTISE(contact_world, std_msgs::Float32MultiArray, "/head/contact_world");
			__ADVERTISE(contact_short, std_msgs::Float32MultiArray, "/head/contact_short");
            __ADVERTISE(contact_distance, std_msgs::Float32MultiArray, "/head/contact_distance");
			__IT_ADVERTISE(cam[0], "/platform/cam0");
			__IT_ADVERTISE(cam[1], "/platform/cam1");
			__IT_ADVERTISE(cam[2], "/platform/cam2");

			//	subscribe
			string topic;

			//	to neck
			topic = topic_root + "/head/neck_cmd";
			cout << "subscribe: " << topic << endl;
			input.neck.sub = output.h_ros->subscribe(
				topic.c_str(),
				ROS_RECV_QUEUE_SIZE,
				&Whiskeye_ModelPlugin::callback_neck,
				this
				);

			//	to theta
			topic = topic_root + "/head/theta_cmd";
			cout << "subscribe: " << topic << endl;
			input.theta.sub = output.h_ros->subscribe(
				topic.c_str(),
				ROS_RECV_QUEUE_SIZE,
				&Whiskeye_ModelPlugin::callback_theta,
				this
				);

			//	to cmd_vel
			topic = topic_root + "/body/cmd_vel";
			cout << "subscribe: " << topic << endl;
			input.cmd_vel.sub = output.h_ros->subscribe(
				topic.c_str(),
				ROS_RECV_QUEUE_SIZE,
				&Whiskeye_ModelPlugin::callback_cmd_vel,
				this
				);

			//	to cmd_pos
			topic = topic_root + "/body/cmd_pos";
			cout << "subscribe: " << topic << endl;
			input.cmd_pos.sub = output.h_ros->subscribe(
				topic.c_str(),
				ROS_RECV_QUEUE_SIZE,
				&Whiskeye_ModelPlugin::callback_cmd_pos,
				this
				);
		}

		void callback_neck(const std_msgs::Float32MultiArray::ConstPtr& msg)
		{
			//	validate input
			if (msg->data.size() != ROBOT_NECK_COUNT)
			{
				__COUT_W << "input /neck wrong size and is ignored" << endl;
				return;
			}

			//	store
			for (int i=0; i<ROBOT_NECK_COUNT; i++)
				input.neck.cmd[i] = msg->data[i] - state.neck_offset[i];
		}

		void callback_theta(const std_msgs::Float32MultiArray::ConstPtr& msg)
		{
			//	validate input
			if (msg->data.size() != ROBOT_WHISKER_COUNT)
			{
				__COUT_W << "input /theta wrong size and is ignored" << endl;
				return;
			}

			//	store
			for (int i=0; i<ROBOT_WHISKER_COUNT; i++)
				input.theta.cmd[i] = msg->data[i];
		}

		void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg)
		{
			//	store
			input.cmd_vel.vel[0] = msg->linear.x;
			input.cmd_vel.vel[1] = msg->linear.y;
			input.cmd_vel.vel[2] = msg->angular.z;

			//	set mode
			SetPosePIDs(false);
		}

		void callback_cmd_pos(const std_msgs::Float32MultiArray::ConstPtr& msg)
		{
			if (msg->data.size() != 3)
				__ERROR("cmd_pos must have three elements (x, y, theta)");

			//	adjust cmd_pos_theta_offset so that this new theta is as
			//	close as possible to the current one
			while (true)
			{
				double theta = msg->data[2] + state.cmd_pos_theta_offset;
				double dtheta = theta - input.cmd_pos.pos[2];

				if (dtheta > M_PI)
				{
					state.cmd_pos_theta_offset -= M_PI * 2.0;
					continue;
				}

				if (dtheta < -M_PI)
				{
					state.cmd_pos_theta_offset += M_PI * 2.0;
					continue;
				}

				break;
			}

			//	store
			input.cmd_pos.pos[0] = msg->data[0];
			input.cmd_pos.pos[1] = msg->data[1];
			input.cmd_pos.pos[2] = msg->data[2] + state.cmd_pos_theta_offset;

			//	set mode
			SetPosePIDs(true);
		}

/*
		//	function not currently used
		void Reset()
		{
			cout << "Reset()" << endl;
		}

		//	function not currently used
		void OnWorldUpdate(const common::UpdateInfo & _info)
		{
			__UNUSED(_info);
			cout << "OnWorldUpdate()" << endl;
		}
*/
		void LoadImage(shared_ptr<sensors::CameraSensor>& cam, sensor_msgs::Image& msg)
		{
			int32_t w = cam->GAZEBO_CAMERA_SENSOR_IMAGE_WIDTH();
			int32_t h = cam->GAZEBO_CAMERA_SENSOR_IMAGE_HEIGHT();
			msg.encoding = "rgb8";
			msg.width = w;
			msg.height = h;
			msg.step = w * 3;
			msg.data.resize(msg.step * h);
			memcpy(&msg.data[0], cam->GAZEBO_CAMERA_SENSOR_IMAGE_DATA(), msg.data.size());
		}

		void OnCamUpdate(sensors::SensorPtr sensor)
		{
			//	get index
			int index = (sensor == sens.cam[0]) ? 0 : ((sensor == sens.cam[1]) ? 1 : 2);

			//	cast
			shared_ptr<sensors::CameraSensor> cam = dynamic_pointer_cast<sensors::CameraSensor> (sensor);

			//	load and publish
			sensor_msgs::Image& msg = output.cam[index].msg;
			LoadImage(cam, msg);
			output.cam[index].pub.publish(msg);
		}

		void OnIMUUpdate(sensors::SensorPtr sensor)
		{
			//	cast
			shared_ptr<sensors::ImuSensor> imu = dynamic_pointer_cast<sensors::ImuSensor> (sensor);

			//	read IMU
			VECTOR3 acc = imu->GAZEBO_IMU_SENSOR_LINEAR_ACCELERATION();

			//	store output
			output.imu_body.msg.linear_acceleration.x = acc[0];
			output.imu_body.msg.linear_acceleration.y = acc[1];
			output.imu_body.msg.linear_acceleration.z = acc[2];

			//	offer 50Hz
			offer_50Hz();
		}

		void OnContactUpdate(sensors::SensorPtr sensor)
		{
			//	cast
			shared_ptr<sensors::ContactSensor> contact = dynamic_pointer_cast<sensors::ContactSensor> (sensor);

			//	sense
			msgs::Contacts contacts = contact->Contacts();
			if (contacts.contact_size())
			{
				//	identify sensor
				int row = -1, col = -1;
				for (int r=0; r<ROBOT_ROW_COUNT; r++)
				{
					for (int c=0; c<ROBOT_COL_COUNT; c++)
					{
						if (sensor == sens.contact[r][c])
						{
							row = r;
							col = c;
							break;
						}
					}
					if (row != -1)
						break;
				}
				if (row == -1)
				{
					__COUT_W << "sensor not found" << endl;
				}
				else
				{
					//cout << row << ", " << col << ": " << contacts.contact_size() << endl;

					//	get first contact in array
					physics::Contact c;
					c = contacts.contact(0);

    				//cout << "--------\nname: " << c.collision2->GetLink()->GetName() << endl;

					//	one of the collision objects will be a whisker - which one?
					physics::LinkPtr link = c.collision1->GetLink();
					if (link != state.link_whisker[row][col])
						link = c.collision2->GetLink();
					if (link != state.link_whisker[row][col])
						__ERROR("whisker collision did not involve whisker");

					//	get its current pose in WORLD
					const WORLD_POSE& whisker_pose = link->GAZEBO_WORLD_POSE();
					//cout << "pose: " << pose << endl;

					//	get transform back to initial pose of this whisker (initial
					//	pose of all whiskers is [0 0 0 0 0 0], see initialisation)
					WORLD_POSE inv_whisker_pose = __POSE_GET_INVERSE(whisker_pose);

					//	convolve with transform back to canonical whisker pose
					inv_whisker_pose = inv_whisker_pose * state.inverse_initial_whisker_pose[row][col];

					//	get pose of head link in WORLD
					const WORLD_POSE& head_pose = state.link_head->GAZEBO_WORLD_POSE();

					//	get transform back to initial pose of HEAD (initial pose of
					//	all members is [0 0 0 0 0 0])
					WORLD_POSE inv_head_pose = __POSE_GET_INVERSE(head_pose);

					//	convolve with transform back to canonical head pose
					inv_head_pose = inv_head_pose * state.inverse_initial_neck_pose[2];

					//	collate total effect of all contacts on this whisker
					double x = 0.0;
					double y = 0.0;
                        
                    //cout << endl;
                    //cout << "XXXXXXXXXXXXXXXXXX Whisker: " << row*4+col << " XXXXXXXXXXXXXXXXXXXXXXX" << endl;
                    double closest = DBL_MAX;
                    VECTOR3 closest_pos;
                    if (contacts.contact_size() > 0)
                    {
                        closest_pos = c.positions[0];
                    }
                    double normalize = 1.0;
					//	for each contact
					for (int i=0; i<contacts.contact_size(); i++)
					{
						//	get contact
                        // Ith contact
						c = contacts.contact(i);
                        //cout << "Number of contacts: " << c.count << endl;
                        //cout << "========= " << i << " =========" << endl;

						//	for each contact element
						for (int j=0; j<c.count; j++)
						{
							//	get contact data
							VECTOR3 pos_WORLD = c.positions[j];
							VECTOR3 norm_WORLD = c.normals[j];
							double depth = c.depths[j];
							
							//	add to output message
							int32_t n = output.contact_world.msg.data.size();
							output.contact_world.msg.data.resize(n + 6);
							float* p_contact_world = &output.contact_world.msg.data[n];
							*(p_contact_world++) = __VECTOR3_X(pos_WORLD);
							*(p_contact_world++) = __VECTOR3_Y(pos_WORLD);
							*(p_contact_world++) = __VECTOR3_Z(pos_WORLD);
							*(p_contact_world++) = __VECTOR3_X(norm_WORLD);
							*(p_contact_world++) = __VECTOR3_Y(norm_WORLD);
							*(p_contact_world++) = __VECTOR3_Z(norm_WORLD);
							/*
							cout << link->GetName()
								<< " : "
								<< pos_WORLD
								<< ", "
								<< norm_WORLD
								<< ", "
								<< depth
								<< endl;
								*/

							//	build world pose from position
                            WORLD_POSE pose_WORLD(pos_WORLD, QUATERNION());

							//	transform contact data into canonical head frame
							VECTOR3 pos_HEAD = __POSE_POS(pose_WORLD * inv_head_pose);
							VECTOR3 norm_HEAD = __POSE_ROT(inv_head_pose).RotateVector(norm_WORLD);
							norm_HEAD *= depth;

							//	add to output message
							n = output.contact_head.msg.data.size();
							output.contact_head.msg.data.resize(n + 6);
							float* p_contact_head = &output.contact_head.msg.data[n];
							*(p_contact_head++) = __VECTOR3_X(pos_HEAD);
							*(p_contact_head++) = __VECTOR3_Y(pos_HEAD);
							*(p_contact_head++) = __VECTOR3_Z(pos_HEAD);
							*(p_contact_head++) = __VECTOR3_X(norm_HEAD);
							*(p_contact_head++) = __VECTOR3_Y(norm_HEAD);
							*(p_contact_head++) = __VECTOR3_Z(norm_HEAD);

							//cout << pos_HEAD << endl;
    
                            if (depth <= closest)
                            {
                                closest = depth;
                                closest_pos = pos_HEAD;
                            }
                        
							//	transform contact data into canonical frame
							VECTOR3 pos_CANON = __POSE_POS(pose_WORLD * inv_whisker_pose);
							VECTOR3 norm_CANON = __POSE_ROT(inv_whisker_pose).RotateVector(norm_WORLD);

							//	report
							//cout << pos << "  @@@  " << norm.GetLength() << endl;

							//	scale normal by depth
							norm_CANON *= depth;

							//	adjust by whisker length
							double z = __VECTOR3_Z(pos_CANON) + ROBOT_WHISKER_Z_OFF;
							norm_CANON *= 1.0 / z;

							//	scale by gain
							norm_CANON *= ROBOT_WHISKER_GAIN;

							//	accumulate
							x += __VECTOR3_X(norm_CANON);
							y += __VECTOR3_Y(norm_CANON);

                            normalize++;
						}
					}
                    //cout << "closest: " << closest << endl;
                    //cout << closest_pos << endl;
                    //cout << "================" << endl;
                    // Publish the short version of the contact points
                    float* c_short = &output.contact_short.msg.data[3*(row*4+col)];
                    *(c_short++) = __VECTOR3_X(closest_pos); 
                    *(c_short++) = __VECTOR3_Y(closest_pos); 
                    *(c_short++) = __VECTOR3_Z(closest_pos); 

					//	get sim time
					//common::Time t = state.model->GetWorld()->GetSimTime();
					//cout << t << endl;

					//	store sensor output
					//
					//	we effectively report through all the samples of each packet (10
					//	samples, usually) the value generated by the contact that occurs
					//	most recently on that whisker before the packet is sent. that's
					//	far from ideal, might want to do some work on this.
					    //
					    //  Olivers hack for the scaling problem of xy values
					    /*
					    if (contacts.contact_size() > 0)
					    {
						x = ((x / normalize) + 1) / 2.0;
						y = ((y / normalize) + 1) / 2.0;
					    }
					    
					    if (x > 1.0 || y > 1.0)
					    {
						cout << "PROBLEM x: " << x << " y: " << y << endl;
					    }
					    if (x < 0.0 || y < 0.0)
					    {
						cout << "NEGATIVE x: " << x << " y: " << y << endl;
					    }
					    */
					struct WhiskerXY* w = (struct WhiskerXY*) &output.bridge_u.msg.xy.data[0];
					for (int i=0; i<ROBOT_FS_SCALE; i++)
					{
						w->xy[i][row][col].x = x;
						w->xy[i][row][col].y = y;
					}


					//	store sensor output
					//
					//	this is a separate output, different from what the robot generates,
					//	that just provides one sample of the most recent data. same info,
					//	different format.
					float* xy = (float*) &output.xy.msg.data[2 * (row*4+col)];
					xy[0] = x;
					xy[1] = y;

					//state.model->GetWorld()->SetPaused(true);
                    
                    
				    // Fill the distances array containing one value for each whisker
				    float* wcd = &output.contact_distance.msg.data[0];
				    for (int i = 0; i < ROBOT_WHISKER_COUNT; i++)
				    { 
								*(wcd++) = i;
				    }
				    //cout << "Whisker " << (row*4+col) << " touching something" << endl; 
				}
			}

			//	offer 50Hz
			offer_50Hz();
		}

		void offer_50Hz()
		{
			/*
				This function is called by every 50Hz sensor. When
				all 50Hz sensors have updated, we can run a 50Hz
				control cycle. We then set sensors_updated to 0.
				Therefore, sensors_updated should always be 0 when
				a physics update runs, which we check to confirm
				synchrony.
			*/

			//	report
			//cout << "sensor " << state.sensors_updated << " @ " << get_sim_time() << endl;

			//	one more sensor updated
			state.sensors_updated++;

			//	if that's all of them
			if (state.sensors_updated == NUM_50HZ_SENSORS)
			{
				//	run the control cycle
				On50Hz();

				//	clear the count
				state.sensors_updated = 0;
			}
		}

		void ReadJointAngles()
		{
			//	this function reads in the current (measured) joint angles from
			//	all the robot joints, and stores them in the outgoing message

			//	neck
			int j = 0;
			for (int t=0; t<ROBOT_FS_SCALE; t++)
				for (int i=0; i<ROBOT_NECK_COUNT; i++)
					output.bridge_u.msg.neck.data[j++] = state.joints_neck[i].joint->GAZEBO_GET_JOINT_POS(0) + state.neck_offset[i];

			//	whiskers
			j = 0;
			for (int t=0; t<ROBOT_FS_SCALE; t++)
				for (int r=0; r<ROBOT_ROW_COUNT; r++)
					for (int c=0; c<ROBOT_COL_COUNT; c++)
						output.bridge_u.msg.theta.data[j++] = state.joints_whisker[r][c].joint->GAZEBO_GET_JOINT_POS(0);
		}

		void TestDriveNeck()
		{
			//	test drive
			static double t;

			double x = sin(t * 2.0 * M_PI * 0.2);
			x = 0.6 + x * 0.3;
			double y = sin(t * 2.0 * M_PI * 0.1);
			y = 0.0 + y * 0.78;
			for (int i=0; i<2; i++)
				input.neck.cmd[i] = x;
			input.neck.cmd[2] = y;

			//	test drive
			t += 0.02;
		}

		void TestDriveWhiskers()
		{
			//	test drive
			static double t;

			double z = sin(t * 2.0 * M_PI * 0.5);
			z = 0.0 + z * 0.78;
			for (int i=0; i<24; i++)
				input.theta.cmd[i] = z;

			//	test drive
			t += 0.02;
		}

		void On50Hz()
		{
			//	shutdown?
			if (!output.h_ros->ok())
				__ERROR("ROS was shutdown");

			//	test drive
			//TestDriveNeck();
			//TestDriveWhiskers();

/*
			//	if the simulation is paused, we sometimes continue to get
			//	these calls - I'm not clear why, or how Gazebo is configured
			//	to synchronize across different objects. in any case, we for
			//	now just detect the paused state and ignore the update
			if (state.model->GetWorld()->IsPaused())
			{
				//	simulation is paused, so do not step model or controllers
//				cout << "." << endl;
				return;
			}
*/

////////////////	OUTPUT

//	can do it this way, but now we read it direct from pose control linkage
#if 0
			//	read robot position (God's odometry)
			const WORLD_POSE& pose = state.link_body->GAZEBO_WORLD_POSE();
			math::Quaternion rot = pose.rot;
			output.pose.msg.x = pose.pos.x;
			output.pose.msg.y = pose.pos.y;
			output.pose.msg.theta = rot.GetYaw();
#endif

			//	read robot position (odometry from pose control linkage)
			output.pose.msg.x = state.joints_pose[0].joint->GAZEBO_GET_JOINT_POS(0);
			output.pose.msg.y = state.joints_pose[1].joint->GAZEBO_GET_JOINT_POS(0);
			output.pose.msg.theta = state.joints_pose[2].joint->GAZEBO_GET_JOINT_POS(0);

			//	read joint angles
			ReadJointAngles();

			//	publish
			output.bumper.pub.publish(output.bumper.msg);
			output.imu_body.pub.publish(output.imu_body.msg);
			output.pose.pub.publish(output.pose.msg);
			output.bridge_u.pub.publish(output.bridge_u.msg);
			output.xy.pub.publish(output.xy.msg);
			output.contact_head.pub.publish(output.contact_head.msg);
            output.contact_distance.pub.publish(output.contact_distance.msg);
			output.contact_world.pub.publish(output.contact_world.msg);
            output.contact_short.pub.publish(output.contact_short.msg);

			//	zero x/y so we can fill them again next time
			struct WhiskerXY* w = (struct WhiskerXY*) &output.bridge_u.msg.xy.data[0];
			memset(w, 0, sizeof(WhiskerXY));
			memset(&output.xy.msg.data[0], 0, 2 * ROBOT_WHISKER_COUNT * sizeof(float));

            // zero the contact short message
            float* c = &output.contact_short.msg.data[0];
            memset(c, 0, sizeof(float));
            memset(&output.contact_short.msg.data[0], 0, 3* ROBOT_WHISKER_COUNT * sizeof(float));

			//	empty contact_head/contact_world for refilling
			output.contact_head.msg.data.resize(0);
			output.contact_world.msg.data.resize(0);

////////////////	INPUT

			//	spin
			ros::spinOnce();

//	NB: I don't like using the string "joint_desc.name" to select these, but
//	the API doesn't seem to define any alternative route...
#define __SETPOS(joint_desc, angle) \
	do { bool ret = state.controller->SetPositionTarget(joint_desc.name, angle); if (!ret) __COUT_W << "joint not found" << endl; } while(false)

//	NB: I don't like using the string "joint_desc.name" to select these, but
//	the API doesn't seem to define any alternative route...
#define __SETVEL(joint_desc, vel) \
	do { bool ret = state.controller->SetVelocityTarget(joint_desc.name, vel); if (!ret) __COUT_W << "joint not found" << endl; } while(false)

			//	neck drive
			for (int i=0; i<ROBOT_NECK_COUNT; i++)
			{
				__SETPOS(state.joints_neck[i], input.neck.cmd[i]);
			}

			//	whisker drive
			for (int i=0; i<ROBOT_WHISKER_COUNT; i++)
				__SETPOS(state.joints_whisker[0][i], input.theta.cmd[i]);

			//	pose drive
			if (state.using_cmd_pos)
			{
				//	pose drive
				for (int i=0; i<3; i++)
				{
					__SETPOS(state.joints_pose[i], input.cmd_pos.pos[i]);
				}
			}

			else
			{
				//	rotate pose drive from robot frame (as delivered) into world frame (as actioned)
				double theta = output.pose.msg.theta;
				double cx = cos(theta);
				double sx = sin(theta);
				double vx = input.cmd_vel.vel[0];
				double vy = input.cmd_vel.vel[1];
				input.cmd_vel.vel[0] = cx * vx - sx * vy;
				input.cmd_vel.vel[1] = sx * vx + cx * vy;

				//	pose drive
				for (int i=0; i<3; i++)
				{
					__SETVEL(state.joints_pose[i], input.cmd_vel.vel[i]);
					input.cmd_vel.vel[i] = 0.0; // in case ROS inputs stop coming, let's not go to the icy wastes
				}
			}
		}

	private:

		//	state
		struct State
		{
			State() :
				model(NULL),
				sensors_connected(0),
				sensors_updated(0),
				using_cmd_pos(false),
				have_selected_using_cmd_pos(false),
				cmd_pos_theta_offset(0.0)
			{
				neck_offset[0] = -M_PI / 2.0;
				neck_offset[1] = 0.0;
				neck_offset[2] = 0.0;
			}

			//	top-level objects
			physics::ModelPtr model;
			physics::JointControllerPtr controller;

			//	link objects
			physics::LinkPtr link_body;
			physics::LinkPtr link_head;
			physics::LinkPtr link_whisker[ROBOT_ROW_COUNT][ROBOT_COL_COUNT];

			//	joint objects
			JointDesc joints_pose[3];
			JointDesc joints_neck[ROBOT_NECK_COUNT];
			JointDesc joints_whisker[ROBOT_ROW_COUNT][ROBOT_COL_COUNT];

			//	sensor connections
			event::ConnectionPtr connection[MAX_SENSOR_COUNT];
			uint32_t sensors_connected;
			uint32_t sensors_updated;

			//	zero on each joint means "as configured in the CAD"; we don't really
			//	have control over this (MJP does it) and it may not agree with the
			//	canonical zero points; we fix this, using an offset here. in principle,
			//	we could do the same with whiskers, but currently MJP == canonical zero.
			double neck_offset[ROBOT_NECK_COUNT];

			//	transforms from initial whisker poses to canonical pose
			WORLD_POSE inverse_initial_whisker_pose[ROBOT_ROW_COUNT][ROBOT_COL_COUNT];

			//	and from poses of neck linkages to canonical
			WORLD_POSE inverse_initial_neck_pose[3];

			//	state of input interface
			bool using_cmd_pos;
			bool have_selected_using_cmd_pos;
			double cmd_pos_theta_offset;
		}
		state;

		//	input
		struct Input
		{
			Input()
			{
			}

			struct Neck
			{
				Neck()
				{
					cmd[0] = cmd[1] = M_PI / 4.0;
					cmd[2] = 0.0;
				}

				ros::Subscriber sub;
				double cmd[ROBOT_NECK_COUNT];
			}
			neck;

			struct Theta
			{
				Theta()
				{
					for (int i=0; i<ROBOT_WHISKER_COUNT; i++)
						cmd[i] = 0.0;
				}

				ros::Subscriber sub;
				double cmd[ROBOT_WHISKER_COUNT];
			}
			theta;

			struct CmdVel
			{
				CmdVel()
				{
					for (int i=0; i<3; i++)
						vel[i] = 0.0;
				}

				ros::Subscriber sub;
				double vel[3];
			}
			cmd_vel;

			struct CmdPos
			{
				CmdPos()
				{
					for (int i=0; i<3; i++)
						pos[i] = 0.0;
				}

				ros::Subscriber sub;
				double pos[3];
			}
			cmd_pos;
		}
		input;

		//	output
		struct Output
		{
			Output()
				:
				h_ros(NULL)
			{
			}

			ros::NodeHandle* h_ros;

			struct Bumper
			{
				Bumper()
				{

				}

				ros::Publisher pub;
				std_msgs::Bool msg;
			}
			bumper;

			struct IMU
			{
				IMU()
				{

				}

				ros::Publisher pub;
				sensor_msgs::Imu msg;
			}
			imu_body;

			struct Pose
			{
				Pose()
				{

				}

				ros::Publisher pub;
				geometry_msgs::Pose2D msg;
			}
			pose;

			struct Cam
			{
				Cam()
				{

				}

				image_transport::Publisher pub;
				sensor_msgs::Image msg;
			}
			cam[3];

			struct BridgeU
			{
				BridgeU()
				{
					msg.neck.data.resize(ROBOT_NECK_COUNT * ROBOT_FS_SCALE);
					msg.theta.data.resize(ROBOT_WHISKER_COUNT * ROBOT_FS_SCALE);
					msg.xy.data.resize(2 * ROBOT_WHISKER_COUNT * ROBOT_FS_SCALE);
					msg.physical.data = false;
				}

				ros::Publisher pub;
				whiskeye_msgs::bridge_u msg;
			}
			bridge_u;

			struct XY
			{
				XY()
				{
					msg.data.resize(2 * ROBOT_WHISKER_COUNT);
				}

				ros::Publisher pub;
				std_msgs::Float32MultiArray msg;
			}
			xy;

            struct ContactShort
            {
                ContactShort()
                {
                    msg.data.resize(3 * ROBOT_WHISKER_COUNT);// This array contains the xyz positions of the contact points closest to the surface
                }
                ros::Publisher pub;
                std_msgs::Float32MultiArray msg; 
            }
            contact_short;

			struct ContactHead
			{
				ContactHead()
				{
				}

				ros::Publisher pub;
				std_msgs::Float32MultiArray msg;
			}
			contact_head;

			struct ContactWorld
			{
				ContactWorld()
				{
				}

				ros::Publisher pub;
				std_msgs::Float32MultiArray msg;
			}
			contact_world;

            struct ContactDistance
            {
                ContactDistance()
                {
                    msg.data.resize(ROBOT_WHISKER_COUNT);
                }
                ros::Publisher pub;
                std_msgs::Float32MultiArray msg;
            }
            contact_distance;
		}
		output;

		//	sensors
		struct Sensors
		{
			Sensors() :
				imu_body(NULL)
			{
			}

			sensors::SensorPtr imu_body;
			sensors::SensorPtr cam[3];
			sensors::SensorPtr contact[ROBOT_ROW_COUNT][ROBOT_COL_COUNT];
		}
		sens;

	};

	GZ_REGISTER_MODEL_PLUGIN(Whiskeye_ModelPlugin)
}



