#include <cloud/navigation/path_planning/path_planning.hpp>

#include <rapp-robots-api/navigation/navigation.hpp>
#include <rapp-robots-api/localization/localization.hpp>
#include <rapp-robots-api/vision/vision.hpp>
#include <rapp-robots-api/communication/communication.hpp>
#include <cloud/vision/hazard_detection/hazard_detection.hpp>
#include <rapp/cloud/service_controller/service_controller.hpp>
#include <rapp/cloud/asio/asio_http/asio_http.hpp>

#include <sstream>
#include <cstdio>
#include <sys/stat.h>
#include <unistd.h>
#include <chrono>
#include <math.h>  

rapp::robot::localization rapp_localization;
rapp::robot::vision rapp_vision;
rapp::robot::navigation rapp_navigation;
rapp::robot::communication rapp_communication(0, NULL);
bool mute = true;


rapp::object::pose handle_localization_position(rapp::cloud::service_controller &ctrl, std::vector<float> &head_angles, rapp::object::pose &last_pose){
	rapp::object::picture::Ptr picture_ptr;
	rapp::object::point diff_pose;
	rapp::object::pose current_pose;
	rapp::object::pose estimated_pose;
	float last_camera_angle;
	float camera_angle_diff;
	std::vector<std::string> joint_names;
	std::vector<float> joint_angles;

	std::vector<std::vector<float>> testing_matrix;
	std::vector<float> testing_vector;

	testing_vector.push_back(1);
	testing_vector.push_back(0);
	testing_vector.push_back(0);
	testing_vector.push_back(1);

	joint_names.clear();
	joint_names.push_back("head_yaw");
	joint_angles.clear();
	joint_angles.push_back(0);
	float x=0;
	float y=0;
	float z=0;
	float current_camera_angle=0;

	auto vis_odom_callback = [&](rapp::object::pose got_pose)
    {estimated_pose = got_pose;};
	for(unsigned int i=0; i<head_angles.size();i++){
		x=0;
		y=0;
		z=0;
		joint_angles.at(0) = head_angles.at(i)*3.14/180;//set new head angle

		rapp_navigation.move_joint(joint_names,joint_angles,0.5);//move head to next angle
		sleep(2);
		picture_ptr = rapp_vision.capture_image(0,3,".jpg");
		
		// rapp_localization.pose_from_matrix(rapp_navigation.get_transform("rgb_head_1", 1), current_pose.pose);
		// std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
  //   	std::chrono::system_clock::duration tp = now.time_since_epoch();
		// current_pose.header.stamp_ = std::chrono::duration_cast<std::chrono::nanoseconds>(tp);
		// std::cout<<"q_x: "<<current_pose.pose.orientation.x<<std::endl;
		// std::cout<<"q_y: "<<current_pose.pose.orientation.y<<std::endl;
		// std::cout<<"q_z: "<<current_pose.pose.orientation.z<<std::endl;
		// std::cout<<"q_w: "<<current_pose.pose.orientation.w<<std::endl;
		// std::cout<<"point_x: "<<current_pose.pose.position.x<<std::endl;
		// std::cout<<"point_y: "<<current_pose.pose.position.y<<std::endl;
		// std::cout<<"point_z: "<<current_pose.pose.position.z<<std::endl;

		testing_matrix = rapp_navigation.get_transform("rgb_head_1", 1);
		for (int i = 0; i < testing_matrix.size(); i++)
		{
		    for (int j = 0; j < testing_matrix[i].size(); j++)
		    {
		        std::cout << testing_matrix[i][j]<<" ";
		    }std::cout <<"\n";
		}
		std::cout << std::endl;
		for(unsigned int j=0; j<testing_vector.size();j++){
			x = x + testing_matrix[0][j]*testing_vector.at(j);
		//				std::cout<<"x: "<<testing_matrix[0][j]<< " * "<<testing_vector.at(j) <<std::endl;

			y = y + testing_matrix[1][j]*testing_vector.at(j);
		//				std::cout<<"y: "<<testing_matrix[1][j]<< " * "<<testing_vector.at(j) <<std::endl;

			z = z + testing_matrix[2][j]*testing_vector.at(j);
		//				std::cout<<"z: "<<testing_matrix[2][j]<< " * "<<testing_vector.at(j) <<std::endl;

		}
		//	std::cout<<"x: "<<x<<std::endl;
		//	std::cout<<"y: "<<y<<std::endl;
		//	std::cout<<"z: "<<z<<std::endl;

		current_camera_angle = atan2(y,x);
		std::cout<<"current_camera_angle: "<<current_camera_angle<<std::endl;
		camera_angle_diff = last_camera_angle - current_camera_angle;
		last_camera_angle = current_camera_angle;

		rapp_localization.pose_from_matrix(testing_matrix, current_pose);
		diff_pose.x = last_pose.position.x - current_pose.position.x;
		diff_pose.y = last_pose.position.y - current_pose.position.y;
		diff_pose.z = camera_angle_diff;
		last_pose = current_pose;
		std::cout<<"x: "<<diff_pose.x<<std::endl;
		std::cout<<"y: "<<diff_pose.y<<std::endl;
		std::cout<<"z: "<<diff_pose.z<<std::endl;

		/// ZMIENIC TYP MAKE CALL
		//ctrl.make_call<rapp::cloud::VIS_ODOM>(picture_ptr, diff_pose, vis_odom_callback);
	}


	return estimated_pose;
}
bool calculare_robot_pose(rapp::object::pose &camera_estimated_pose,rapp::object::pose &end_pose){
	rapp::object::pose camera_in_robot_pose;
	rapp_localization.pose_from_matrix(rapp_navigation.get_transform("rgb_head_1", 0), camera_in_robot_pose);
	rapp_localization.multiply_poses(camera_estimated_pose, camera_in_robot_pose, end_pose);
	return true;
}
int visionLocalization(rapp::cloud::service_controller &ctrl){
	rapp::object::pose estimated_pose;
	rapp::object::pose_stamped camera_in_robot_pose;
	rapp::object::pose end_pose;
	rapp::object::pose last_pose;
	rapp::object::pose_stamped current_pose;
	rapp::object::pose_stamped robot_in_map_pose;

	current_pose = rapp_navigation.get_global_pose();

	rapp_navigation.take_predefined_posture("Zero", 0.5);
	sleep(2);
	std::vector<rapp::object::picture::Ptr> picture_vector;
	std::vector<float> head_angles;
	head_angles.clear();
	for(int i=0; i<7; i++){
		head_angles.push_back(90-i*30);
	}
		estimated_pose = handle_localization_position(ctrl,head_angles, last_pose);
	
	//rapp_navigation.move_to(0.5,0,1.57);
	//estimated_pose = handle_localization_position(ctrl,head_angles);
	calculare_robot_pose(estimated_pose, end_pose);

	//rapp_navigation.set_global_pose(end_pose);
	return 0;
}

int main(int argc, char** argv){
	rapp::cloud::platform_info info = {"192.168.18.180", "9001", "rapp_token"}; 
    rapp::cloud::service_controller ctrl(info);
visionLocalization(ctrl);
return 0;
}