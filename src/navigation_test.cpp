#include "rapp-robots-api/navigation/navigation.hpp"
#include "rapp/objects/pose_stamped/pose_stamped.hpp"
#include <iostream>
#include <unistd.h>
int main(int argc, char ** argv){

	rapp::robot::navigation nav;

	std::vector<std::string> joint_names;
	joint_names.clear();
	std::vector<float> joint_angles;
	joint_angles.clear();
	joint_names.push_back("head_pitch");
	joint_angles.push_back(-1); // turn head in pitch to -1 rad orientation
	joint_names.push_back("head_yaw");
	joint_angles.push_back(1.57); // turn head in yaw to 1.57 rad orientation

	nav.move_joint(joint_names, joint_angles);

	rapp::object::pose_stamped pose_start;
	 // pose_mid;
	std::vector<rapp::object::pose_stamped> poses_list;
	poses_list.clear();
	// push the start pose for moveAlongPath method
	poses_list.push_back(pose_start);
	// determine mid pose for moveAlongPath method
	rapp::object::pose_stamped pose_mid(rapp::object::header(0,rapp::object::time(0,0),"/map"), rapp::object::pose(rapp::object::point(1,1,0),rapp::object::quaternion(0,0,0,1))); 
	poses_list.push_back(pose_mid);
	// determine goal pose for moveAlongPath method
	rapp::object::pose_stamped pose_goal(rapp::object::header(0,rapp::object::time(0,0),"/map"), rapp::object::pose(rapp::object::point(2,0,0),rapp::object::quaternion(0,0,0,1))); 

	poses_list.push_back(pose_goal);
	// move robot via combined path
	nav.move_along_path(poses_list);
	//move robot to pose given in his frame
	nav.move_to(-2,0,0);
	// set robot speed to linear = 0.5, angular = 0.4
	nav.move_vel(0.5,0.4);
	// wait 8 secs
	sleep(5);
	// stop move
	nav.move_stop();
	return 0;
}
