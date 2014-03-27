/*
 * ros_arm_cartesian_control.cpp
 *
 *  Created on: Nov 29, 2012
 *      Author: matthias
 */

#include "mcr_manipulation_utils/ros_urdf_loader.h"
#include "arm_cartesian_control.h"

#include <sensor_msgs/JointState.h>
#include <kdl/kdl.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <brics_actuator/JointVelocities.h>
#include <tf/transform_listener.h>

KDL::Chain arm_chain;
KDL::Chain base_chain;
KDL::Chain full_chain;
std::vector<boost::shared_ptr<urdf::JointLimits> > joint_limits;

KDL::JntArray joint_positions;
std::vector<bool> joint_positions_initialized;

KDL::Twist targetVelocity;

KDL::ChainIkSolverVel* ik_solver;
Eigen::MatrixXd weight_ts;
Eigen::MatrixXd weight_js;

ros::Publisher cmd_vel_publisher;
ros::Publisher base_cart_vel_publisher;
tf::TransformListener *tf_listener;

bool active = false;
ros::Time t_last_command;

brics_actuator::JointVelocities jointMsg;
brics_actuator::JointVelocities armJointMsg;
geometry_msgs::Twist baseMsg;

std::string root_name = "DEFAULT_CHAIN_ROOT";


void jointstateCallback(sensor_msgs::JointStateConstPtr joints) {
    //TODO: note: full_chain, arm_chain and base_chain are all used here.
    //TODO: Why the nested loop? Could use two loops, not nested.
	for (unsigned i = 0; i < joints->position.size(); i++) {

		const char* joint_uri = joints->name[i].c_str();

		for (unsigned int j = 0; j < arm_chain.getNrOfJoints(); j++) {
        	const char* chainjoint =
					full_chain.getSegment(j+base_chain.getNrOfJoints()).getJoint().getName().c_str();
            
			if (chainjoint != 0 && strcmp(chainjoint, joint_uri) == 0) {
				joint_positions.data[j+base_chain.getNrOfJoints()] = joints->position[i];
				joint_positions_initialized[j+base_chain.getNrOfJoints()] = true;
			}
		}
	}
    //TODO: add base "joint" states, this is currently just zero, could use odom.
    for (unsigned int i = 0; i < base_chain.getNrOfJoints(); i++) {
        joint_positions.data[i] = 0.0;
        joint_positions_initialized[i] = true;
    }

}

void ccCallback(geometry_msgs::TwistStampedConstPtr desiredVelocity) {

	for (size_t i = 0; i < joint_positions_initialized.size(); i++) {
		if (!joint_positions_initialized[i]) {
			std::cout << "joints not initialized" << std::endl;
			return;
		}
	}

	if (!tf_listener) return;

	try {
	  geometry_msgs::Vector3Stamped linear_in;
	  geometry_msgs::Vector3Stamped linear_out;
	  linear_in.header = desiredVelocity->header;
	  linear_in.vector = desiredVelocity->twist.linear;
	  tf_listener->transformVector(root_name, linear_in, linear_out);

	  geometry_msgs::Vector3Stamped angular_in;
	  geometry_msgs::Vector3Stamped angular_out;
	  angular_in.header = desiredVelocity->header;
	  angular_in.vector = desiredVelocity->twist.angular;
	  tf_listener->transformVector(root_name, angular_in, angular_out);

	  targetVelocity.vel.data[0] = linear_out.vector.x;
	  targetVelocity.vel.data[1] = linear_out.vector.y;
	  targetVelocity.vel.data[2] = linear_out.vector.z;

	  targetVelocity.rot.data[0] = angular_out.vector.x;
	  targetVelocity.rot.data[1] = angular_out.vector.y;
	  targetVelocity.rot.data[2] = angular_out.vector.z;

	  t_last_command = ros::Time::now();
	} catch(...) {
	  ROS_ERROR("Could not transform frames %s -> %s", desiredVelocity->header.frame_id.c_str(), root_name.c_str());
	}
	active = true;
}


void init_ik_solver() {

	if(ik_solver != 0) {
		return;
	}

	//ik_solver = new KDL::ChainIkSolverVel_wdls(arm_chain);
    ik_solver = new KDL::ChainIkSolverVel_wdls(full_chain);
	//weight_ts.resize(6, 6);
    //TODO why is that 6 by 6?
	//weight_ts.setIdentity();

	//weight_ts(0, 0) = 1;
	//weight_ts(1, 1) = 1;
	//weight_ts(2, 2) = 10;
	//weight_ts(3, 3) = 0.0001;
	//weight_ts(4, 4) = 0.0001;
	//weight_ts(5, 5) = 0.0001;
	//((KDL::ChainIkSolverVel_wdls*) ik_solver)->setWeightTS(weight_ts);

	//weight_js = (Eigen::MatrixXd::Identity(arm_chain.getNrOfJoints(),
	//		arm_chain.getNrOfJoints()));
	weight_js = (Eigen::MatrixXd::Identity(full_chain.getNrOfJoints(),
                     full_chain.getNrOfJoints()));
        weight_js(0,0) = 0.1;
	weight_js(1,1) = 0.1;
	weight_js(2,2) = 0.1;
	weight_js(3,3) = 1;
	weight_js(4,4) = 1;
	weight_js(5,5) = 1;
        weight_js(6,6) = 1;
        weight_js(7,7) = 1;
        ((KDL::ChainIkSolverVel_wdls*) ik_solver)->setWeightJS(weight_js);


	((KDL::ChainIkSolverVel_wdls*) ik_solver)->setLambda(10000.0);
}

void init_joint_msgs() {
	//joint_positions_initialized.resize(arm_chain.getNrOfJoints(), false);
	joint_positions_initialized.resize(full_chain.getNrOfJoints(), false);
    //jointMsg.velocities.resize(arm_chain.getNrOfJoints());
    jointMsg.velocities.resize(full_chain.getNrOfJoints());
    // Later the none dummy joints are copied back to armJointMsg for publishing.
    armJointMsg.velocities.resize(arm_chain.getNrOfJoints());

    //for (unsigned int i = 0; i < arm_chain.getNrOfSegments(); i++) {
    for (unsigned int i = 0; i < arm_chain.getNrOfSegments(); i++) {
		//jointMsg.velocities[i].joint_uri =
		//		arm_chain.getSegment(i).getJoint().getName();
        int j = i + base_chain.getNrOfJoints();
        ROS_ERROR("J, I values are %d, %d", j, i);
	ROS_ERROR("arm_chain nr of segments: %d", arm_chain.getNrOfSegments());
		jointMsg.velocities[j].joint_uri = full_chain.getSegment(j).getJoint().getName();
        jointMsg.velocities[j].unit = "s^-1 rad";
	ROS_ERROR("jointMsg [%d] j uri: %s", j, jointMsg.velocities[j].joint_uri.c_str());
	ROS_ERROR("unit: %s", jointMsg.velocities[j].unit.c_str());
	}
    // the base joints have diff units. KDL JointType Enum is in form Txxx, Rxxx or None
    for (unsigned int i = 0; i < base_chain.getNrOfJoints(); i++) {
        const char* joint_type = base_chain.getSegment(i).getJoint().getTypeName().c_str();
        if ( strncmp(joint_type, "Txxxx", 1) == 0) {
            jointMsg.velocities[i].unit = "m s^-1";
        } else {
            jointMsg.velocities[i].unit = "s^-1 rad";
        }
        //jointMsg.velocities[i].joint_uri = base_chain.getSegment(i).getJoint().getName();
        jointMsg.velocities[i].joint_uri = "dummy_base_link";
	ROS_ERROR("ADDED dummy joint %d with units %s", i, jointMsg.velocities[i].unit.c_str() );
    }

   for (unsigned int i = 0; i < jointMsg.velocities.size(); i++) {
        ROS_ERROR("I value",  i);
        ROS_ERROR("velocities size: %d", jointMsg.velocities.size());
        ROS_ERROR("joint uri: %s", jointMsg.velocities[i].joint_uri.c_str());
        ROS_ERROR("unit: %s", jointMsg.velocities[i].unit.c_str());
   }
}

void publishJointVelocities(KDL::JntArrayVel& joint_velocities) {
    //TODO first 3 joints are 
    
   for (unsigned int i = 0; i < joint_velocities.qdot.rows(); i++) {
        ROS_ERROR("I value %d",  i);
        ROS_ERROR("velocities rows: %d", joint_velocities.qdot.rows());
        ROS_ERROR("velocities size: %d", jointMsg.velocities.size());
        ROS_ERROR("qdot value %.5f", joint_velocities.qdot(i));
        ROS_ERROR("joint uri: %s", jointMsg.velocities[i].joint_uri.c_str());
        ROS_ERROR("unit: %s", jointMsg.velocities[i].unit.c_str());
   }


	//for (unsigned int i=0; i<joint_velocities.qdot.rows(); i++) {
    for (unsigned int i=0; i<arm_chain.getNrOfJoints(); i++) {
        int j = i + base_chain.getNrOfJoints();
        if (j > joint_velocities.qdot.rows()){
            //break;
           ROS_ERROR("j is too large");
        }
        armJointMsg.velocities[i] = jointMsg.velocities[j];
	armJointMsg.velocities[i].value = joint_velocities.qdot(j);
        armJointMsg.velocities[i].unit = "s^-1 rad";
		ROS_DEBUG("%s: %.5f %s", armJointMsg.velocities[i].joint_uri.c_str(), 
			  armJointMsg.velocities[i].value, armJointMsg.velocities[i].unit.c_str());
		if (isnan(armJointMsg.velocities[i].value)) {
			ROS_ERROR("invalid joint velocity: nan");
			return;
		}
		if (fabs(armJointMsg.velocities[i].value) > 1.0) {
			ROS_ERROR("invalid joint velocity: too fast");
			return;
		}
	}
	cmd_vel_publisher.publish(armJointMsg);

    // TODO PUBLISH TO BASE
     baseMsg.angular.z = joint_velocities.qdot(0);
     baseMsg.linear.x = joint_velocities.qdot(1);
     baseMsg.linear.y = joint_velocities.qdot(2);
     
     ROS_ERROR("Publishing to base z:%.5f, x:%.5f, y:%.5f",
	baseMsg.angular.z, baseMsg.linear.x, baseMsg.linear.y);
     base_cart_vel_publisher.publish(baseMsg);
}


void stopMotion() {
    // Stop the Arm
	for (unsigned int i = 0; i < armJointMsg.velocities.size(); i++) {
		armJointMsg.velocities[i].value = 0.0;

	}
	cmd_vel_publisher.publish(armJointMsg);
    //TODO  Stop the Base


}


bool watchdog() {

	double watchdog_time = 0.3;
	if (active==false) {
		return false;
	}

	ros::Time now = ros::Time::now();

	ros::Duration time = (now - t_last_command);

	if ( time > ros::Duration(watchdog_time) ) {
		active = false;
		stopMotion();
		return false;
	}

	return true;
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "arm_cartesian_control");
	ros::NodeHandle node_handle("~");
	tf_listener = new tf::TransformListener();

	double rate = 50;

	//TODO: read from param
 	std::string velocity_command_topic = "joint_velocity_command";
	std::string joint_state_topic = "/joint_states";
	std::string cart_control_topic = "cartesian_velocity_command";
        std::string base_command_topic = "/cmd_vel";        

	std::string tooltip_name = "DEFAULT_CHAIN_TIP";

	if (!node_handle.getParam("root_name", root_name)) {
		ROS_ERROR("No parameter for root_name specified");
		return -1;
        }
	ROS_INFO("Using %s as chain root [param: root_name]", root_name.c_str());

	if (!node_handle.getParam("tip_name", tooltip_name)) {
		ROS_ERROR("No parameter for tip_name specified");
		return -1;
	}
	ROS_INFO("Using %s as tool tip [param: tip_name]", tooltip_name.c_str());


	//load URDF model
	ROS_URDF_Loader loader;
	loader.loadModel(node_handle, root_name, tooltip_name, arm_chain, joint_limits);

    // Add three joints base_chain, and combine with the arm_chain
    printf("Will now add three segments to base_chain");
    base_chain.addSegment(KDL::Segment(KDL::Joint("base_joint_z", KDL::Joint::RotZ)));
    base_chain.addSegment(KDL::Segment(KDL::Joint("base_joint_x", KDL::Joint::TransX)));
    base_chain.addSegment(KDL::Segment(KDL::Joint("base_joint_y", KDL::Joint::TransY),
                          KDL::Frame(KDL::Vector(0.143, 0.0, 0.0))));

    printf("creating full chain from base_chain");
    full_chain = KDL::Chain(base_chain);
    printf("adding arm chain");
    full_chain.addChain(arm_chain);

	//init
	//joint_positions.resize(arm_chain.getNrOfJoints());
    joint_positions.resize(full_chain.getNrOfJoints());

	init_ik_solver();

	init_joint_msgs();

	//fk_solver = new KDL::ChainFkSolverPos_recursive(arm_chain);
	//jnt2jac = new KDL::ChainJntToJacSolver(arm_chain);



	//register publisher
	cmd_vel_publisher = node_handle.advertise<brics_actuator::JointVelocities>(
			velocity_command_topic, 1);
	base_cart_vel_publisher = node_handle.advertise<geometry_msgs::Twist>(
			base_command_topic, 1);

	//register subscriber
	ros::Subscriber sub_joint_states = node_handle.subscribe(joint_state_topic,
			1, jointstateCallback);
	ros::Subscriber sub_cc = node_handle.subscribe(cart_control_topic, 1,
			ccCallback);


	//arm_cc::Arm_Cartesian_Control control(&arm_chain, ik_solver);
	arm_cc::Arm_Cartesian_Control control(&full_chain, ik_solver);
    std::vector<double> upper_limits;
	std::vector<double> lower_limits;
    // the base chain links are dummy joints, with inf limits.
    for (unsigned int i=0; i<base_chain.getNrOfJoints(); i++) {
        upper_limits.push_back(20000.0);
        lower_limits.push_back(-20000.0);
    }
	for (unsigned int i=0; i<joint_limits.size(); i++) {
		upper_limits.push_back(joint_limits[i]->upper);
		lower_limits.push_back(joint_limits[i]->lower);
	}
	control.setJointLimits(lower_limits, upper_limits);

	//KDL::JntArrayVel cmd_velocities(arm_chain.getNrOfJoints());
    KDL::JntArrayVel cmd_velocities(full_chain.getNrOfJoints());


	//loop with 50Hz
	ros::Rate loop_rate(rate);

	while (ros::ok()) {

		ros::spinOnce();

		if(watchdog()) {
			control.process(1/rate, joint_positions, targetVelocity, cmd_velocities);

			publishJointVelocities(cmd_velocities);
		}


		loop_rate.sleep();
	}

	delete tf_listener;

	return 0;
}

