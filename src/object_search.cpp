#include <rapp-robots-api/navigation/navigation.hpp>
#include <rapp-robots-api/localization/localization.hpp>
#include <rapp-robots-api/vision/vision.hpp>
#include <rapp-robots-api/communication/communication.hpp>
#include <rapp/objects/picture/picture.hpp>

#include <rapp/cloud/service_controller/service_controller.hpp>
#include <rapp/cloud/vision/object_detection/object_detection.hpp>
#include <cloud/navigation/path_planning/path_planning.hpp>
#include <rapp/cloud/vision/object_detection/object_detection.hpp>
#include <cloud/navigation/visual_localization/visual_localization.hpp>

//read hazard xml file
#include <boost/foreach.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>

rapp::robot::localization rapp_localization;
rapp::robot::vision rapp_vision;
rapp::robot::navigation rapp_navigation;
rapp::robot::communication rapp_communication(0, NULL);
bool mute = true;

typedef std::vector<rapp::object::pose> middle_poses_vector;

class GlobalLocalizationHandler {
   public:
	    rapp::object::pose handle_localization_position(rapp::cloud::service_controller &ctrl, std::vector<float> &head_angles, rapp::object::pose &last_pose, int localization_service_id);
	    bool calculare_robot_pose(rapp::object::pose &camera_estimated_pose,rapp::object::pose &end_pose);
	    bool handleGlobalLocalization_rgb_cam(rapp::cloud::service_controller &ctrl, int localization_service_id);
	    int globalLocalization_qr_code(std::string *QRmap_path);
	    bool handleGlobalLocalization_qr_code(std::string qr_map_file_path);
	    GlobalLocalizationHandler();  // This is the constructor

	
};
	GlobalLocalizationHandler::GlobalLocalizationHandler(){

	}

	rapp::object::pose GlobalLocalizationHandler::handle_localization_position(rapp::cloud::service_controller &ctrl, std::vector<float> &head_angles, rapp::object::pose &last_pose, int localization_service_id){
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
	bool GlobalLocalizationHandler::calculare_robot_pose(rapp::object::pose &camera_estimated_pose,rapp::object::pose &end_pose){
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


	bool GlobalLocalizationHandler::handleGlobalLocalization_rgb_cam(rapp::cloud::service_controller &ctrl, int localization_service_id){

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

	int GlobalLocalizationHandler::globalLocalization_qr_code(std::string *QRmap_path){
		int status;
		rapp::object::pose new_pose;
		rapp::object::qr_code_map map;
		rapp::object::qr_code_3d detected_qrcodes;


		rapp::object::picture::Ptr picture_ptr  = rapp_vision.capture_image (0, 3, ".png");
		picture_ptr->save("./localization_picture.png");
		std::cout<<"after CAPTURE"<<std::endl;
		std::vector<std::vector<float>> robotToCameraMatrix = rapp_navigation.get_transform("rgb_head_1",0);

		double camera_top_matrix[3][3];
		camera_top_matrix[0][0] =1373.916183;
			camera_top_matrix[0][2] = 655.543569;
			camera_top_matrix[1][1] = 1371.191854;
			camera_top_matrix[1][2] = 462.618540;
			camera_top_matrix[2][2] = 1.0;
			camera_top_matrix[0][1] = 0.000000;
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
	bool GlobalLocalizationHandler::handleGlobalLocalization_qr_code(std::string qr_map_file_path){

		int localization_status = 2;
		bool camera_joint_range = false;
		std::vector<std::string> move_joints_names;

		move_joints_names.clear();
		move_joints_names.push_back("rgb_head_1");
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
middle_poses_vector read_middle_poses_from_file(std::istream &is){
   // populate tree structure pt
    //std::istream is = ifs;
    using boost::property_tree::ptree;
    ptree pt;
             std::cout<< "0"<<std::endl;
             //is<<""; 
    read_xml(is, pt);
             std::cout<< "1"<<std::endl;

    // traverse pt
    middle_poses_vector ans;
    BOOST_FOREACH( ptree::value_type const& v, pt.get_child("poses") ) {
             std::cout<< "1"<<std::endl;

        if( v.first == "pose" ) {
            rapp::object::pose f;

            f.position.x = v.second.get<float>("position.x");
            std::cout<< "f.position.x :"<<f.position.x<<std::endl;
            f.position.y = v.second.get<float>("position.y");
            std::cout<< "f.position.y :"<<f.position.y<<std::endl;
            f.position.z = v.second.get<float>("position.z");
            std::cout<< "f.position.z :"<<f.position.z<<std::endl;
            f.orientation.x = v.second.get<float>("orientation.x");
            std::cout<< "f.orientation.yx :"<<f.orientation.x<<std::endl;

            f.orientation.y = v.second.get<float>("orientation.y");
            std::cout<< "f.orientation.y :"<<f.orientation.y<<std::endl;
            f.orientation.z = v.second.get<float>("orientation.z");
            std::cout<< "f.orientation.z :"<<f.orientation.z<<std::endl;
            f.orientation.w = v.second.get<float>("orientation.w");
            std::cout<< "f.orientation.w :"<<f.orientation.w<<std::endl;

            ans.push_back(f);
        }
    }
              std::cout<< "end"<<std::endl;

    return ans;

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
void status_cb(int status) {
    if (status != 0)
        std::cout << "Error!\n";
}

bool lookForObject(rapp::cloud::service_controller &ctrl, std::string token, std::vector<rapp::object::point> &point_in_cam_vec){
	std::vector<float> head_angles;
	std::vector<std::string> joint_names;
	rapp::object::point point_in_cam;
	joint_names = {"head_yaw", "head_pitch"};
	head_angles = {-90.0f, -60.0f, -30.0f, 0.0f, 30.0f, 60.0f, 90.0f};
	rapp::object::picture::Ptr picture_ptr;
	rapp::robot::vision::camera_info cam = rapp_vision.load_camera_info(0);
   	ctrl.make_call<rapp::cloud::object_detection_clear_models>(status_cb);
    std::vector<std::string> models = {"mint"};
    ctrl.make_call<rapp::cloud::object_detection_load_models>(models, status_cb);

    float cx = cam.K[2];
    float cy = cam.K[5];
    float fx = cam.K[0];
    float fy = cam.K[4];
	for(unsigned int i = 0; i < head_angles.size(); i++){
        rapp_navigation.move_joint(joint_names, {head_angles.at(i)*(float) 3.14/180, 0.5f});
		sleep(3);
		picture_ptr = rapp_vision.capture_image(0,3,".png");
		picture_ptr->save("./find_object_pic.png");
		ctrl.make_call<rapp::cloud::object_detection_find_objects>(picture_ptr, 10, [&](std::vector<std::string> names, std::vector<rapp::object::point> centers, std::vector<float> scores, int result) {
			for (unsigned int i = 0; i < names.size(); i++){
				std::cout<<"found object: "<< names.at(i)<<std::endl;
				
				rapp_communication.text_to_speech(names[i]);
		    	std::cout << names[i] << "(" << centers[i].x << "," << centers[i].y << ")\n";
	        
	            float u = centers[i].x;
	            float v = centers[i].y;

	            //float a1 = -atan2(u-cx, fx);
	            //float a2 = atan2(v-cy, fy);
	            
		    	float cam_z = 0.5;
	            float cam_x = (u-cx)/fx*cam_z;
		    	float cam_y = (v-cy)/fy*cam_z;

	            std::cout << "In cam: " << cam_x << "," << cam_y << "," << cam_z << "\n";
	            point_in_cam.x = cam_z;
	            point_in_cam.y = -cam_x;
	            point_in_cam.z = -cam_y;
				
				point_in_cam_vec.push_back(point_in_cam);
			}
		});
		if (point_in_cam_vec.size()>0){
			std::cout<<"look for found!"<<std::endl;

			return true;
		}
	}
	return true;
}
// return: 0 - collision free path not found, 1 - found, 2 - move_along error, 3 - lookForObject error, 4 - not found
int handleOneMiddlePose(rapp::cloud::service_controller &ctrl,std::string token, rapp::object::pose goal_pose_rapp, rapp::object::pose_stamped current_pose, std::string ObsMap_path, std::string ObsMap_name, std::vector<rapp::object::point> &point_in_cam_vec){

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
	std::vector<rapp::object::pose_stamped> next_pose;

	planner_response = cloudPlanPath(ctrl,current_pose,goal_pose_rapp_stamp, ObsMap_name);

	// move and check for object
	bool move_along_status;
	bool lookForObject_status;
	for(unsigned int i = 1; i < planner_response.path.size(); i++){
		next_pose.clear();
		next_pose.push_back(planner_response.path.at(i));
		lookForObject_status = lookForObject(ctrl, token, point_in_cam_vec);
		if (!lookForObject_status){
			return 3;
		}
		if (point_in_cam_vec.size() > 0){
			std::cout<<"handle found!"<<std::endl;
			return 1;
		}
		if (point_in_cam_vec.size() > 0){
			std::cout<<"handle found!"<<std::endl;
			return 4;
		}
		move_along_status = rapp_navigation.move_along_path(next_pose);
			std::cout<<"move status: "<<move_along_status<<std::endl;

		if (move_along_status){
			return 2;
		}

	}
	return 0;
}
rapp::object::point getObjectPose(rapp::object::point &box_in_camera_point){

	rapp::object::point box_in_map_point;

	std::vector<float> box_in_camera_vector;
	box_in_camera_vector.clear();
	box_in_camera_vector.push_back(box_in_camera_point.x);
	box_in_camera_vector.push_back(box_in_camera_point.y);
	box_in_camera_vector.push_back(box_in_camera_point.z);
	box_in_camera_vector.push_back(1);

	std::vector<std::vector<float>> camera_map_transform;
	camera_map_transform = rapp_navigation.get_transform("rgb_head_1",1);
	for (unsigned int i = 0; i < camera_map_transform.size(); i++)
		{
	    for (unsigned int j = 0; j < camera_map_transform[i].size(); j++)
	    {
	    	std::cout << camera_map_transform[i][j]<<" ";
	    }
	    std::cout <<"\n";
	}
	for(unsigned int i=0; i<camera_map_transform.at(0).size(); i++ ){
		box_in_map_point.x = box_in_map_point.x + camera_map_transform.at(0).at(i) * box_in_camera_vector.at(i);
		box_in_map_point.y = box_in_map_point.y + camera_map_transform.at(1).at(i) * box_in_camera_vector.at(i);
		box_in_map_point.z = box_in_map_point.z + camera_map_transform.at(2).at(i) * box_in_camera_vector.at(i);
		std::cout<<"in loop"<<std::endl;
	}
	std::cout<<"box_map_x: "<<box_in_map_point.x<<std::endl;
	std::cout<<"box_map_y: "<<box_in_map_point.y<<std::endl;
	std::cout<<"box_map_z: "<<box_in_map_point.z<<std::endl;
	return box_in_map_point;
}

int main(int argc, char** argv){

std::string poses_xml_file_path;
bool use_QR_MAP;
std::string QR_map_file_path;
std::string obstacle_map_path;
std::string obstacle_map_name;
std::string token;

GlobalLocalizationHandler GLhandler;
if (argc!=6){
	std::cout<<"Too less arguments. Setting default arguments: "<<std::endl;
	std::cout<<"poses_xml_file_path: ./my_poses.xml\n"<< "mute: 0\n"<< "use_QR_MAP: 0\n"<<"QR_map_file_path: ./qr_map.xml\n"<< "token: rapp_token"<<std::endl;
	poses_xml_file_path = "./my_poses.xml";
	mute = false;
	use_QR_MAP = false;
	QR_map_file_path= "./qr_map.xml";
	obstacle_map_path = "./";
	obstacle_map_name = "obstacle_map";
	token = "rapp_token";
}else{
	std::string poses_xml_file(argv[1]);
	poses_xml_file_path = poses_xml_file;
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

// [1] - check requested files
if (!check_file(poses_xml_file_path)){
	std::cout<<"Poses file not found: "<< poses_xml_file_path <<std::endl;	
	return 0;		
}
if(use_QR_MAP){
	if (!check_file(QR_map_file_path)){
	std::cout<<"QR map file not found: "<<QR_map_file_path<<std::endl;	
	return 0;		
	}
}
rapp::cloud::platform_info info = {"192.168.18.186", "9001", token}; 
rapp::cloud::service_controller ctrl(info);
// [2] - get user request
rapp_navigation.take_predefined_posture("Zero", 0.5);
rapp_communication.text_to_speech("Hello I am Electron. What should I look for?");
sleep(3);
std::vector<std::string> word_dictionary = {"mint"};
//std::string spotted_word = rapp_communication.word_spotting(word_dictionary);
std::string spotted_word = "mint";
rapp_communication.text_to_speech("I will look for "+spotted_word);

// [3] - read middle poses from file
std::ifstream poses_path( poses_xml_file_path );
middle_poses_vector middle_poses;
middle_poses = read_middle_poses_from_file(poses_path);

// [4] - global localization
bool localization_status;
int id = -1;
// if (use_QR_MAP){
// 	localization_status = GLhandler.handleGlobalLocalization_qr_code(QR_map_file_path);
// }else{
// 	ctrl.make_call<rapp::cloud::visual_localization_init>("map1.xml", [&](int ret_id){id = ret_id;});
// 	localization_status = GLhandler.handleGlobalLocalization_rgb_cam(ctrl,id);
// }
// if (!localization_status){
// 	rapp_communication.text_to_speech("Global localization error. Aborting task");
// 	return 0;
// }
// [5] - request collision free path to the first middlepose
rapp::object::pose_stamped current_pose;

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
rapp::object::point object_pose;
int handleOneMiddlePose_status;
std::vector<rapp::object::point> point_in_cam_vec;
for(unsigned int i = 0; i < middle_poses.size(); i++){
	current_pose = rapp_navigation.get_global_pose();
	handleOneMiddlePose_status = handleOneMiddlePose(ctrl, token, middle_poses.at(i), current_pose, obstacle_map_path, obstacle_map_name, point_in_cam_vec);
	std::cout<<"handlestatus: "<<handleOneMiddlePose_status<<std::endl;
	if (handleOneMiddlePose_status == 1){
		rapp_communication.text_to_speech("I found "+spotted_word);
		object_pose = getObjectPose(point_in_cam_vec.at(0));
		rapp_navigation.point_arm(object_pose.x,object_pose.y,object_pose.z+0.15);
		rapp_communication.text_to_speech("It is here");
		return 0;
	}
	if (handleOneMiddlePose_status == 0){
		rapp_communication.text_to_speech("I cant localize myself in this room");
		return 0;
	}
	if (handleOneMiddlePose_status == 4){
		rapp_communication.text_to_speech("I cant find "+spotted_word+" in this pose. Moving to the next middlepose");
		return 0;
	}
	// if (use_QR_MAP){
	// 	localization_status = GLhandler.handleGlobalLocalization_qr_code(QR_map_file_path);
	// }else{
	// 	ctrl.make_call<rapp::cloud::visual_localization_init>("map1.xml", [&](int ret_id){id = ret_id;});
	// 	localization_status = GLhandler.handleGlobalLocalization_rgb_cam(ctrl,id);
	// }
	// if (!localization_status){
	// 	rapp_communication.text_to_speech("Global localization error. Aborting task");
	// 	return 0;
	// }
}

return 0;
}
