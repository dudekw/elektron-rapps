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

rapp::robot::localization rapp_localization;
rapp::robot::vision rapp_vision;
rapp::robot::navigation rapp_navigation;
rapp::robot::communication rapp_communication(0, NULL);
bool mute = true;

int globalLocalization(std::string *QRmap_path){
	int status;
	rapp::object::pose new_pose;
	rapp::object::qr_code_map map;
	rapp::object::qr_code_3d detected_qrcodes;


	rapp::object::picture::Ptr picture_ptr  = rapp_vision.capture_image (0, 3, ".png");
	picture_ptr->save("/home/robot/pictureTest.png");
	std::cout<<"after CAPTURE"<<std::endl;
	std::vector<std::vector<float>> robotToCameraMatrix = rapp_navigation.get_transform("rgb_head_1",0);

	double camera_top_matrix[3][3];
	camera_top_matrix[0][0] = 1776.709300;
		camera_top_matrix[0][2] = 665.442605;
		camera_top_matrix[1][1] = 1790.305076;
		camera_top_matrix[1][2] = 420.572422;
		camera_top_matrix[2][2] = 1.0;
		camera_top_matrix[0][1] = 0.0;
		camera_top_matrix[1][0] = 0.0;
		camera_top_matrix[2][0] = 0.0;
		camera_top_matrix[2][1] = 0.0;

	float lendmarkSize = 0.16f;
	detected_qrcodes = rapp_vision.qr_code_detection(picture_ptr, robotToCameraMatrix, camera_top_matrix, lendmarkSize);

	if(!mute)
	rapp_communication.text_to_speech("I found");
	int t = detected_qrcodes.number_of_qr_codes;
	std::stringstream ss;
	ss << t;
	if(!mute)
	rapp_communication.text_to_speech(ss.str());
	if(!mute)
	rapp_communication.text_to_speech("QRcodes");

	if (detected_qrcodes.number_of_qr_codes > 0){
        std::cout<<"Found QR_code label: |"<< detected_qrcodes.qr_message.at(0)<<"|"<<std::endl;

        std::cout<<"QR_map_path: |"<<QRmap_path<<"|"<<std::endl;
		map = rapp_localization.load_qr_code_map(QRmap_path);

		std::string label = map.labels.at(0);
        std::cout<<"label[0]: |"<<label<<"|"<<std::endl;
		
		new_pose = rapp_localization.qr_code_localization(detected_qrcodes,robotToCameraMatrix, map);
        std::cout<<"pose_x: |"<<new_pose.position.x<<"|"<<std::endl;

		bool setPose_status = rapp_navigation.set_global_pose(new_pose);
		std::cout << setPose_status<<std::endl;
		if (!setPose_status){
			status = 0;
		}else{
			if(!mute)
			rapp_communication.text_to_speech("Setting pose status FAILED");

			std::cout <<"Setting pose status FAILED"<<std::endl;

			status = 1;
		}
	}else{
		if(!mute)
		rapp_communication.text_to_speech("NO QRcodes found!");
		if(!mute)
		rapp_communication.text_to_speech("Changing my head position");
		std::cout << "NO QRcodes found!"<<std::endl;
		status = 2;
	}
	return status;
	}

bool check_file (const std::string& name) {
	struct stat buffer;   
	return (stat (name.c_str(), &buffer) == 0); 
}
rapp::object::planned_path cloudPlanPath(rapp::cloud::service_controller &ctrl, rapp::object::pose_stamped  current_pose, rapp::object::pose_stamped  goal_pose_rapp, std::string map_name){

rapp::object::planned_path ret;

	auto callback = [&](rapp::object::planned_path path)
    {
        if (path.plan_found) {
            for(const auto & p : path.path) {
                std::cout << "[" << p.pose.position.x << ", " << p.pose.position.y << "]" << std::endl;
            }
            ret = path;
        } else {
            std::cout << "Path not found\n";
        }
    };

    ctrl.make_call<rapp::cloud::plan_path_2d>(map_name, "NAO", "dijkstra", current_pose, goal_pose_rapp, callback);
return ret;
}

void uploadMap(rapp::cloud::service_controller &ctrl,std::string map_path, std::string map_name){


    // callback lambda UPLOAD MAP
    auto upload_callback = [&](std::string status)
    {std::cout << "upload status: \n" << status << " \n" << std::endl;};
    // upload map
    std::string yaml_file_name  = map_path + "/" + map_name + ".yaml";
    std::string png_file_name  = map_path + "/" + map_name + ".png";
    std::cout<<"yaml: " << yaml_file_name<<std::endl;
    std::cout<<"png: " << png_file_name<<std::endl;
    std::cout<<"map_name: "<<map_name<<std::endl;
    auto yaml_file = rapp::object::yaml(yaml_file_name);
    auto picture_file = rapp::object::picture(png_file_name);

    ///
    ctrl.make_call<rapp::cloud::path_upload_map>(picture_file, yaml_file, map_name, upload_callback);

}

void printPose(rapp::object::pose_stamped  pose){

std::cout<< "POSE:" <<std::endl;
std::cout<< "  x: " << pose.pose.position.x <<std::endl;
std::cout<< "  y: " << pose.pose.position.y <<std::endl;
std::cout<< "  z: " << pose.pose.position.z <<std::endl;

std::cout<< "  ox: " << pose.pose.orientation.x <<std::endl;
std::cout<< "  oy: " << pose.pose.orientation.y <<std::endl;
std::cout<< "  oz: " << pose.pose.orientation.z <<std::endl;
std::cout<< "  ow: " << pose.pose.orientation.w <<std::endl;

}

int main (int argc, char ** argv ) {

	if (argc<14){

	std::cout<<"Too less arguments. Arguments are: "<<std::endl;
	std::cout<<" x, y, z, ox, oy, oz, ow,         X, Y, Z,          mute,     map_file_path,  token"<<std::endl;
	std::cout<<"|hazard detection pose  | hazard detection point|mute speech| path to QR map| API token |"<<std::endl;
	return 0;	
	}
	std::string qr_map_file_path=argv[12];
	if (!check_file(qr_map_file_path)){
	std::cout<<"Map file not found: "<<argv[12]<<std::endl;	
	return 0;		
	}
	int mute_trigger;
	std::istringstream ss_mute_trigger(argv[11]);
	ss_mute_trigger >> mute_trigger;
	if(mute_trigger==1){
		mute = true;
	}else{
		mute=false;
	}
	int localization_status = 2;
	bool camera_joint_range = false;
	std::vector<std::string> move_joints_names;

	move_joints_names.clear();
	move_joints_names.push_back("head_yaw");
	std::vector<float> new_joint_angle;
	rapp_navigation.take_predefined_posture("Zero", 0.5);
	// global localization with Nao head motion
	std::cout<<"setting head start position"<< std::endl;
	new_joint_angle.clear();
	new_joint_angle.push_back((-119*(3.1415/180)));
	while ((localization_status != 0 || camera_joint_range)){
		if(!mute)
		rapp_communication.text_to_speech("Calling global localization");

			std::cout<<"Calling global localization"<< std::endl;

		localization_status = globalLocalization(&qr_map_file_path);
		if (localization_status==1)
		{	
			if(!mute)
			rapp_communication.text_to_speech("Application FAILED because of core agent crash");

			std::cout<<"RApp FAILED because of core agent crash"<< std::endl;
			return 1;	
		}else if (localization_status==0){
			std::cout<<"localization successful"<<std::endl;
		} else if(localization_status == 2){
			bool moveJoint_status = rapp_navigation.move_joint(move_joints_names, new_joint_angle, 0.5);
			sleep(4);
			if (moveJoint_status)
				localization_status = 1;

			new_joint_angle.at(0) = new_joint_angle.at(0)+float(30*(3.14/180));
			std::cout<<"New head position request: "<<new_joint_angle.at(0) <<std::endl;

			if (new_joint_angle.at(0)>119*(3.1415/180)){
				camera_joint_range = true;
				if(!mute)
				rapp_communication.text_to_speech("I can't rotate my head further, give me QRcodes!");
				std::cout<<"Robot head checked whole camera spectrum, Hazard detection ends"<< std::endl;
				return 1;	
			}
		}

	}
	// ----
	// get robot position
	rapp::object::pose_stamped current_pose = rapp_navigation.get_global_pose();
	// ----
	//compose hazard pose and hazard point 
	rapp::object::pose goal_pose_rapp;

	std::vector<float> goal_pose_input;
	float x;
	goal_pose_input.clear();
	for (int i=1; i<=7 ; i++){
	std::istringstream ss1(argv[i]);
	ss1 >> x;
	goal_pose_input.push_back(x);
	std::cout<<"goal pose ["<<i<<"]"<<": "<<x<<std::endl;

	}
	for (int i=8; i<=10 ; i++){
	std::istringstream ss2(argv[i]);
	ss2 >> x;
	goal_pose_input.push_back(x);
	}
	// hazard pose
	goal_pose_rapp.position.x = goal_pose_input.at(0);
	goal_pose_rapp.position.y = goal_pose_input.at(1);
	goal_pose_rapp.position.z = goal_pose_input.at(2);
	goal_pose_rapp.orientation.x = goal_pose_input.at(3);
	goal_pose_rapp.orientation.y = goal_pose_input.at(4);
	goal_pose_rapp.orientation.z = goal_pose_input.at(5);
	goal_pose_rapp.orientation.w = goal_pose_input.at(6);

	// hazard point
	float hazard_point_x = goal_pose_input.at(7);
	float hazard_point_y = goal_pose_input.at(8);
	float hazard_point_z = goal_pose_input.at(9);

	auto nsec = std::chrono::nanoseconds(10);		  	


	//std::chrono::miliseconds ms(10);
	rapp::object::msg_metadata goal_meta(0, nsec, "world" );

	rapp::object::pose_stamped goal_pose_rapp_stamp(goal_meta,goal_pose_rapp);

	// ----

	// CLOUD

	// ----
	std::cout<<"CURRENT POSE:"<<std::endl;
	printPose(current_pose);
	std::cout<<"GOAL POSE:"<<std::endl;

	printPose(goal_pose_rapp_stamp);
	// service controler
	std::string token = argv[15];

    rapp::cloud::platform_info info = {"192.168.18.180", "9001", token}; 
    rapp::cloud::service_controller ctrl(info);

	// upload map
    std::string ObsMap_path(argv[13]);
    std::string ObsMap_name(argv[14]);
	uploadMap(ctrl,ObsMap_path,ObsMap_name);

	// plann path
	rapp::object::planned_path planner_response;

	planner_response = cloudPlanPath(ctrl,current_pose,goal_pose_rapp_stamp, ObsMap_name);


	// move along path
	bool move_along_status = rapp_navigation.move_along_path(planner_response.path);

	// handle failed moveAlongPath

	// ----
	//rapp_navigation.rest("Crouch");

	bool look_at_point_status = rapp_navigation.look_at_point(hazard_point_x, hazard_point_y, hazard_point_z);
	std::cout<<"look at point status: "<< look_at_point_status<<std::endl;
	// handle failed lookAtPoint

	// ----

	if(!mute)
	rapp_communication.text_to_speech("I'm checking the door status now.");
	sleep(5);

/*
	// call detect hazard method
	//
	//
	//
	//    bool door_status = checkDoor();
	//
	//
	std::string token = argv[13];
	rapp::object::picture::Ptr door_picture_ptr  = rapp_vision.capture_image (0, 3 , "png");
	rapp::cloud::service_controller ctrl;
	auto callback = [&](double angle) { std::cout << "Door angle " << angle << std::endl; };
	auto door_check = std::make_shared<rapp::cloud::hazard_detection_door_check>(door_picture_ptr, calback, token);
	if (door_check) {
		ctrl.run_job(door_check);
	}
	//
	// say sth
*/

	if(!mute)
	rapp_communication.text_to_speech("The front door is opened");
	if(!mute)
	rapp_communication.text_to_speech("I need to rest now");
	//  
	//
	//rapp_navigation.rest("Crouch");
	return 0;
}

