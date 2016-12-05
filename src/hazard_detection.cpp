#include <cloud/navigation/path_planning/path_planning.hpp>
#include <cloud/navigation/visual_localization/visual_localization.hpp>

#include <rapp-robots-api/navigation/navigation.hpp>
#include <rapp-robots-api/localization/localization.hpp>
#include <rapp-robots-api/vision/vision.hpp>
#include <rapp-robots-api/communication/communication.hpp>
#include <cloud/vision/hazard_detection/hazard_detection.hpp>
#include <rapp/cloud/service_controller/service_controller.hpp>
#include <rapp/cloud/asio/asio_http/asio_http.hpp>
//read hazard xml file
#include <boost/foreach.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>

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

struct hazard_data
{
    std::string         type;
    unsigned            ID;
    rapp::object::pose 	pose;
    rapp::object::point 	point;
};
 
typedef std::vector<hazard_data> hazard_vector;

hazard_vector read_hazards_from_file(std::istream &is){
   // populate tree structure pt
    //std::istream is = ifs;
    using boost::property_tree::ptree;
    ptree pt;
             std::cout<< "0"<<std::endl;
             //is<<""; 
    read_xml(is, pt);
             std::cout<< "1"<<std::endl;

    // traverse pt
    hazard_vector ans;
    BOOST_FOREACH( ptree::value_type const& v, pt.get_child("hazards") ) {
             std::cout<< "1"<<std::endl;

        if( v.first == "hazard" ) {
            hazard_data f;
             std::cout<< "2"<<std::endl;
            f.type = v.second.get<std::string>("type");
            std::cout<< "TYPE :"<<f.type<<std::endl;

            f.ID = v.second.get<unsigned>("ID");
             std::cout<< "3"<<std::endl;

            f.pose.position.x = v.second.get<float>("pose.position.x");
            std::cout<< "f.pose.position.x :"<<f.pose.position.x<<std::endl;
            f.pose.position.y = v.second.get<float>("pose.position.y");
            std::cout<< "f.pose.position.y :"<<f.pose.position.y<<std::endl;
            f.pose.position.z = v.second.get<float>("pose.position.z");
            std::cout<< "f.pose.position.z :"<<f.pose.position.z<<std::endl;
            f.pose.orientation.x = v.second.get<float>("pose.orientation.x");
            std::cout<< "f.pose.orientation.yx :"<<f.pose.orientation.x<<std::endl;

            f.pose.orientation.y = v.second.get<float>("pose.orientation.y");
            std::cout<< "f.pose.orientation.y :"<<f.pose.orientation.y<<std::endl;
            f.pose.orientation.z = v.second.get<float>("pose.orientation.z");
            std::cout<< "f.pose.orientation.z :"<<f.pose.orientation.z<<std::endl;
            f.pose.orientation.w = v.second.get<float>("pose.orientation.w");
            std::cout<< "f.pose.orientation.w :"<<f.pose.orientation.w<<std::endl;
            f.point.x = v.second.get<float>("point.x");
            std::cout<< "f.point.x :"<<f.point.x<<std::endl;
            f.point.y = v.second.get<float>("point.y");
            std::cout<< "f.point.y :"<<f.point.y<<std::endl;
            f.point.z = v.second.get<float>("point.z");
            std::cout<< "f.point.z :"<<f.point.z<<std::endl;

            ans.push_back(f);
        }
    }
              std::cout<< "end"<<std::endl;

    return ans;

}

rapp::object::pose handle_localization_position(rapp::cloud::service_controller &ctrl, std::vector<float> &head_angles, rapp::object::pose &last_pose, int localization_service_id){
	rapp::object::picture::Ptr picture_ptr;
	rapp::object::point diff_pose;
	rapp::object::point got_pose;

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
		std::cout <<"transf_matrix size: "<<testing_matrix.size()<<std::endl;
		if (testing_matrix.size()==0){
			std::cout <<"transf_matrix size: "<<testing_matrix.size()<<std::endl;
			testing_matrix = rapp_navigation.get_transform("rgb_head_1", 1);
			sleep(1);
		}
		std::cout <<"transf_matrix size: "<<testing_matrix.size()<<std::endl;
		//testing_matrix = rapp_localization.invert_transform(testing_matrix);
		for (unsigned int i = 0; i < testing_matrix.size(); i++)
		{
		    for (unsigned int j = 0; j < testing_matrix[i].size(); j++)
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


		// NAO
		//current_camera_angle = atan2(x,z);
		// Elektron
		current_camera_angle = atan2(y,x);
		std::cout<<"current_camera_angle: "<<current_camera_angle<<std::endl;
		camera_angle_diff = current_camera_angle -last_camera_angle;
		last_camera_angle = current_camera_angle;

		rapp_localization.pose_from_matrix(testing_matrix, current_pose);
		diff_pose.x = last_pose.position.x - current_pose.position.x;
		diff_pose.y = last_pose.position.y - current_pose.position.y;
		diff_pose.z = camera_angle_diff;
		last_pose = current_pose;
		std::cout<<"sent_x: "<<diff_pose.x<<std::endl;
		std::cout<<"sent_y: "<<diff_pose.y<<std::endl;
		std::cout<<"sent_z: "<<diff_pose.z<<std::endl;

		/// ZMIENIC TYP MAKE CALL

		ctrl.make_call<rapp::cloud::visual_localization>(localization_service_id, picture_ptr, diff_pose, [&](rapp::object::point pt, float bel, int st){
		    got_pose.x = pt.x;
		std::cout<<"got_x: "<<pt.x<<std::endl;

		    got_pose.y = pt.y;
		std::cout<<"got_y: "<<pt.y<<std::endl;

		    got_pose.z = pt.z;
		std::cout<<"got_z: "<<pt.z<<std::endl;

		});
   	}
	estimated_pose.position.x = got_pose.x;
	std::cout<<"estimated_camera_x: "<<got_pose.x<<std::endl;
	estimated_pose.position.y = got_pose.y;
	std::cout<<"estimated_camera_y: "<<got_pose.y<<std::endl;
	estimated_pose.position.z = last_pose.position.z;
	std::cout<<"estimated_camera_z: "<<last_pose.position.z<<std::endl;
	std::cout<<"estimated_camera_orientation: "<<got_pose.z<<std::endl;
	estimated_pose.orientation = rapp_localization.quaternion_from_euler(0,0,got_pose.z);
	
	std::cout<<"FROM LOCALIZATION HANDLER: \n"<<std::endl;

		std::cout<<"q_x: "<<estimated_pose.orientation.x<<std::endl;
		std::cout<<"q_y: "<<estimated_pose.orientation.y<<std::endl;
		std::cout<<"q_z: "<<estimated_pose.orientation.z<<std::endl;
		std::cout<<"q_w: "<<estimated_pose.orientation.w<<std::endl;
		std::cout<<"point_x: "<<estimated_pose.position.x<<std::endl;
		std::cout<<"point_y: "<<estimated_pose.position.y<<std::endl;
		std::cout<<"point_z: "<<estimated_pose.position.z<<std::endl;	
	return estimated_pose;
}
bool calculare_robot_pose(rapp::object::pose &camera_estimated_pose,rapp::object::pose &end_pose){
	rapp::object::pose camera_in_robot_pose;
	std::vector<std::vector<float>> camera_to_robot_matrix = rapp_navigation.get_transform("rgb_head_1", 0);
	rapp_localization.pose_from_matrix(camera_to_robot_matrix, camera_in_robot_pose);
	rapp_localization.multiply_poses(camera_estimated_pose, camera_in_robot_pose, end_pose);
	std::cout<<"FROM CALCULATE HANDLER: \n"<<std::endl;

		std::cout<<"q_x: "<<end_pose.orientation.x<<std::endl;
		std::cout<<"q_y: "<<end_pose.orientation.y<<std::endl;
		std::cout<<"q_z: "<<end_pose.orientation.z<<std::endl;
		std::cout<<"q_w: "<<end_pose.orientation.w<<std::endl;
		std::cout<<"point_x: "<<end_pose.position.x<<std::endl;
		std::cout<<"point_y: "<<end_pose.position.y<<std::endl;
		std::cout<<"point_z: "<<end_pose.position.z<<std::endl;

	return true;
}


bool handleGlobalLocalization_rgb_cam(rapp::cloud::service_controller &ctrl, int localization_service_id){

	rapp::object::pose estimated_pose;
	rapp::object::pose_stamped camera_in_robot_pose;
	rapp::object::pose end_pose;
	rapp::object::pose last_pose;
	rapp::object::pose_stamped current_pose;
	rapp::object::pose_stamped robot_in_map_pose;

	current_pose = rapp_navigation.get_global_pose();

	rapp_navigation.take_predefined_posture("Zero", 0.5);
	std::vector<rapp::object::picture::Ptr> picture_vector;
	std::vector<float> head_angles;
	head_angles.clear();
	for(int i=0; i<7; i++){
		head_angles.push_back(90-i*30);
	}
	estimated_pose = handle_localization_position(ctrl,head_angles, last_pose, localization_service_id);
	
	//rapp_navigation.move_to(0.5,0,1.57);
	//estimated_pose = handle_localization_position(ctrl,head_angles);
	calculare_robot_pose(estimated_pose, end_pose);

	rapp_navigation.set_global_pose(end_pose);
	return true;
}

int globalLocalization_qr_code(std::string *QRmap_path){
	int status;
	rapp::object::pose new_pose;
	rapp::object::qr_code_map map;
	rapp::object::qr_code_3d detected_qrcodes;


	rapp::object::picture::Ptr picture_ptr  = rapp_vision.capture_image (0, 3, ".png");
	picture_ptr->save("./localization_picture.png");
	std::cout<<"after CAPTURE"<<std::endl;
	std::vector<std::vector<float>> robotToCameraMatrix = rapp_navigation.get_transform("rgb_head_1",0);

	double camera_top_matrix[3][3];
	camera_top_matrix[0][0] = 182.0992346 / 0.16;
		camera_top_matrix[0][2] = 658.7582;
		camera_top_matrix[1][1] = 185.0952141 / 0.16;
		camera_top_matrix[1][2] = 484.2186;
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
		if (setPose_status){
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
bool plannAndMove(rapp::cloud::service_controller &ctrl,std::string token, rapp::object::pose goal_pose_rapp, rapp::object::pose_stamped current_pose, std::string ObsMap_path, std::string ObsMap_name){

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

	// upload map
    // std::string ObsMap_path(argv[13]);
    // std::string ObsMap_name(argv[14]);
	uploadMap(ctrl,ObsMap_path,ObsMap_name);

	// plann path
	rapp::object::planned_path planner_response;

	planner_response = cloudPlanPath(ctrl,current_pose,goal_pose_rapp_stamp, ObsMap_name);


	// move along path
	bool move_along_status = rapp_navigation.move_along_path(planner_response.path);
	return move_along_status;
}

bool handleGlobalLocalization_qr_code(std::string qr_map_file_path){

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
	new_joint_angle.push_back((-149*(3.1415/180)));
	while ((localization_status != 0 || camera_joint_range)){
		if(!mute)
		rapp_communication.text_to_speech("Calling global localization");

			std::cout<<"Calling global localization"<< std::endl;

		localization_status = globalLocalization_qr_code(&qr_map_file_path);
		if (localization_status==1)
		{	
			if(!mute)
			rapp_communication.text_to_speech("Application FAILED because of core agent crash");

			std::cout<<"RApp FAILED because of core agent crash"<< std::endl;
			return false;	
		}else if (localization_status==0){
			if(!mute)
                        rapp_communication.text_to_speech("localization successful");
			std::cout<<"localization successful"<<std::endl;
		} else if(localization_status == 2){
			new_joint_angle.at(0) = new_joint_angle.at(0)+float(30*(3.14/180));

			if (new_joint_angle.at(0)>119*(3.1415/180)){
				camera_joint_range = true;
				if(!mute)
				rapp_communication.text_to_speech("I can't rotate my head further, give me QRcodes!");
				std::cout<<"Robot head checked whole camera spectrum, Hazard detection ends"<< std::endl;
				return false;	
			}

			bool moveJoint_status = rapp_navigation.move_joint(move_joints_names, new_joint_angle, 0.5);
			// sleep(4);
			if (!moveJoint_status)
				localization_status = 1;

			std::cout<<"New head position request: "<<new_joint_angle.at(0) <<std::endl;

			
		}

	}

return true;
}

bool handleHazard(rapp::cloud::service_controller &ctrl, std::string token, rapp::object::point point_for_hazard, int hazard_ID){
	// handle failed moveAlongPath

	// ----
	//rapp_navigation.rest("Crouch");

	bool look_at_point_status = rapp_navigation.look_at_point(point_for_hazard.x, point_for_hazard.y, point_for_hazard.z);
	std::cout<<"look at point status: "<< look_at_point_status<<std::endl;
	// handle failed lookAtPoint

	// ----




	// call detect hazard method
	//
	//
	//
	//    bool door_status = checkDoor();
	//
	//
    if (hazard_ID == 1){
    	if(!mute)
		rapp_communication.text_to_speech("I'm checking the door status now.");
		sleep(5);
		rapp::object::picture::Ptr door_picture_ptr  = rapp_vision.capture_image (0, 3 , "png");
		door_picture_ptr->save("./Door_picture.png");

		auto callback = [&](double angle) { std::cout << "Door angle " << angle << std::endl;
		if (angle > 0){
		rapp_communication.text_to_speech("The door is opened.");
		sleep(5);
		}else {
		rapp_communication.text_to_speech("The door is closed.");
		sleep(5);
		}
		 };
		rapp::object::picture door_picture = *door_picture_ptr;
		ctrl.make_call<rapp::cloud::hazard_detection_door_check>(door_picture, callback);

	}else if(hazard_ID == 2){
    	if(!mute)
		rapp_communication.text_to_speech("I'm checking the lamp status now.");
		sleep(5);
		rapp::object::picture::Ptr lamp_picture_ptr  = rapp_vision.capture_image (0, 3 , "png");
		lamp_picture_ptr->save("./Lamp_picture.png");
		auto callback = [&](double light_level) { std::cout << "lamp status: " << light_level << std::endl; 

		if (light_level > 0){
		rapp_communication.text_to_speech("The lamp is switched on.");
		sleep(5);
		}else {
		rapp_communication.text_to_speech("The lamp is switched off.");
		sleep(5);
		}

		};
		rapp::object::picture lamp_picture = *lamp_picture_ptr;
		ctrl.make_call<rapp::cloud::hazard_detection_light_check>(lamp_picture, callback);
	}


/*	//
	// say sth
*/return true;
}



int main (int argc, char ** argv ) {
	std::string hazards_xml_file_path;
	bool use_QR_MAP;
	std::string QR_map_file_path;
	std::string obstacle_map_path;
	std::string obstacle_map_name;
	std::string token;
	if (argc!=6){
		std::cout<<"Too less arguments. Setting default arguments: "<<std::endl;
		std::cout<<"hazards_xml_file_path: ./my_hazards.xml\n"<< "mute: 0\n"<< "use_QR_MAP: 0\n"<<"QR_map_file_path: ./qr_map.xml\n"<< "token: rapp_token"<<std::endl;
		hazards_xml_file_path = "./my_hazards.xml";
		mute = false;
		use_QR_MAP = false;
		QR_map_file_path= "./qr_map.xml";
		obstacle_map_path = "./";
		obstacle_map_name = "obstacle_map";
		token = "rapp_token";
	}else{
		std::string hazards_xml_file(argv[1]);
		hazards_xml_file_path = hazards_xml_file;
		if(atoi(argv[2])==1){
			mute = true;
		}else{
			mute=false;
		}
		if(atoi(argv[3])==1){
			use_QR_MAP = true;
		}else{
			use_QR_MAP=false;
		}

		QR_map_file_path= "./qr_map.xml";
		obstacle_map_path = "./";
		obstacle_map_name = "obstacle_map";
		token = "rapp_token";
	}
	if(use_QR_MAP){
		if (!check_file(QR_map_file_path)){
		std::cout<<"QR map file not found: "<<QR_map_file_path<<std::endl;	
		return 0;		
		}
	}
	//compose hazard pose and hazard point 
	std::ifstream hazard_path( hazards_xml_file_path );

	if ( !hazard_path ) {
		std::cout<<"Hazard file path is wrong: "<< hazards_xml_file_path <<std::endl;	
		return 0;
	}
	std::string hazard_path_str = hazards_xml_file_path;
	if (!check_file(hazard_path_str)){
		std::cout<<"Hazard file not found: "<< hazards_xml_file_path <<std::endl;	
		return 0;		
	}

	hazard_vector hazards = read_hazards_from_file(hazard_path);

	rapp::object::pose pose_for_door;
	rapp::object::point point_for_door;
	rapp::object::point point_for_lamp;
	rapp::object::pose pose_for_lamp;
	for(unsigned int i = 0; i < hazards.size(); i++){
		if (hazards.at(i).type == "door"){
			pose_for_door = hazards.at(i).pose;
			point_for_door = hazards.at(i).point;
		}
		if (hazards.at(i).type == "lamp"){
			pose_for_lamp = hazards.at(i).pose;
			point_for_lamp = hazards.at(i).point;
		}
	}
	rapp::cloud::platform_info info = {"192.168.18.186", "9001", token}; 
    rapp::cloud::service_controller ctrl(info);


	// Start RApp FSM

	// 1. Global localization
	bool localization_status = true;
 //   	int id = -1;

 //    if (use_QR_MAP){
	// 	localization_status = handleGlobalLocalization_qr_code(QR_map_file_path);
 //    }else{
 //   		ctrl.make_call<rapp::cloud::visual_localization_init>("map2", [&](int ret_id){id = ret_id;});
	// 	localization_status = handleGlobalLocalization_rgb_cam(ctrl,id);
 //    }

	// rapp::object::pose global_pose;
	// global_pose.position.x = 1;
	// global_pose.position.y = 1;
	// global_pose.position.z = 0;
	// global_pose.orientation.x = 0;
	// global_pose.orientation.y = 0;
	// global_pose.orientation.z = 0;
	// global_pose.orientation.w = 1;
	// rapp_navigation.set_global_pose(global_pose);
	// sleep(4);
 	if (localization_status){
	// get robot position
        rapp::object::pose_stamped current_pose = rapp_navigation.get_global_pose();
        // ----

	// 2. move to door check
	plannAndMove(ctrl, token, pose_for_door, current_pose, obstacle_map_path, obstacle_map_name);

	// 1. Global localization
    if (use_QR_MAP){
		localization_status = handleGlobalLocalization_qr_code(QR_map_file_path);
    }else{
		localization_status = handleGlobalLocalization_rgb_cam(ctrl,id);
    }
 	if (localization_status){

	rapp::object::pose global_pose;
	global_pose.position.x = 1;
	global_pose.position.y = 1;
	global_pose.position.z = 0;
	global_pose.orientation.x = 0;
	global_pose.orientation.y = 0;
	global_pose.orientation.z = 0;
	global_pose.orientation.w = 1;
	rapp_navigation.set_global_pose(global_pose);
	sleep(4);
	// 3. check the door status
	handleHazard(ctrl,token, point_for_door, 1);


 	// get robot position
        current_pose = rapp_navigation.get_global_pose();
        // ----
	// 4. move to lamp check
	plannAndMove(ctrl, token, pose_for_lamp, current_pose, obstacle_map_path, obstacle_map_name);
	
	rapp_navigation.take_predefined_posture("Zero", 0.5);
	// 5. check the lamp status
	handleHazard(ctrl,token, point_for_lamp, 2);

	// if(!mute)
	// rapp_communication.text_to_speech("The front door is opened");
	if(!mute)
		rapp_communication.text_to_speech("My job is done.");
	//  
	//
	rapp_navigation.rest("Crouch");
	return 0;

        }else{

                if(!mute)
                rapp_communication.text_to_speech("Cant localize in the environment, my job is done");
		return 1;
        }


	}else{

		if(!mute)
                rapp_communication.text_to_speech("Cant localize in the environment, my job is done.");
		return 1;
	}

}


