/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: Azmyin Md. Kamal
Date: 01/01/24

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

//* Includes
#include "ros2_orb_slam3/common.hpp"


//* Constructor
MonocularMode::MonocularMode() :Node("mono_node_cpp")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    
    //* Find path to home directory
    homeDir = getenv("HOME");
    // std::cout<<"Home: "<<homeDir<<std::endl;
    
    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  
    
    //* Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();

    // rclcpp::Parameter param4 = this->get_parameter("settings_file_name_arg");
    
  
    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        pass;
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular/";
    }

    // std::cout<<"vocFilePath: "<<vocFilePath<<std::endl;
    // std::cout<<"settingsFilePath: "<<settingsFilePath<<std::endl;
    
    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    // RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());
    
    subexperimentconfigName = "/mono_py_driver/experiment_settings"; // topic that sends out some configuration parameters to the cpp ndoe
    pubconfigackName = "/mono_py_driver/exp_settings_ack"; // send an acknowledgement to the python node
    subImgMsgName = "/mono_py_driver/img_msg"; // topic to receive RGB image messages
    subTimestepMsgName = "/mono_py_driver/timestep_msg"; // topic to receive RGB image messages

    //* subscribe to python node to receive settings
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&MonocularMode::experimentSetting_callback, this, _1));

    //* publisher to send out acknowledgement
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);

    //* subscrbite to the image messages coming from the Python driver node
    subImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, 1, std::bind(&MonocularMode::Img_callback, this, _1));

    //* subscribe to receive the timestep
    subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));

    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("camera_path", 10);
    pointcloud2_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_pointcloud2", 10);
    frame_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("keypoint_render_frame", 10);
    

 
    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
    
}

//* Destructor
MonocularMode::~MonocularMode()
{   
    
    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();
    pass;

}

//* Callback which accepts experiment parameters from the Python node
void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg){
    
    // std::cout<<"experimentSetting_callback"<<std::endl;
    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    // receivedConfig = experimentConfig; // Redundant
    
    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", this->receivedConfig.c_str());

    //* Publish acknowledgement
    auto message = std_msgs::msg::String();
    message.data = "ACK";
    
    std::cout<<"Sent response: "<<message.data.c_str()<<std::endl;
    configAck_publisher_->publish(message);

    //* Wait to complete VSLAM initialization
    initializeVSLAM(experimentConfig);

}

//* Method to bind an initialized VSLAM framework to this node
void MonocularMode::initializeVSLAM(std::string& configString){
    
    // Watchdog, if the paths to vocabular and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    //* Build .yaml`s file path
    
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml"); // Example ros2_ws/src/orb_slam3_ros2/orb_slam3/config/Monocular/TUM2.yaml

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::MONOCULAR; 
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularMode node initialized" << std::endl; // TODO needs a better message
}

//* Callback that processes timestep sent over ROS
void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg){
    // timeStep = 0; // Initialize
    timeStep = time_msg.data;
}

//* Callback to process image message and run SLAM node
void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    // Initialize
    //cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient
    
    //* Convert ROS image to openCV image
    try
    {
        //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
        cv_ptr = cv_bridge::toCvCopy(msg); // Local scope
        
        // DEBUGGING, Show image
        // Update GUI Window
        // cv::imshow("test_window", cv_ptr->image);
        // cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    
    // std::cout<<std::fixed<<"Timestep: "<<timeStep<<std::endl; // Debug
    
    //* Perform all ORB-SLAM3 operations in Monocular mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    //Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep); 
    Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep); 

    std::unique_lock<std::mutex> locker_pose(pose_mutex_);
    pose_buffer_.push(Tcw);
    locker_pose.unlock();
    pose_cv_.notify_one();

    std::unique_lock<std::mutex> locker_image(image_mutex_);
    image_buffer_.push(true);
    locker_image.unlock();
    image_cv_.notify_one();


    std::unique_lock<std::mutex> locker_point(point_mutex_);
    point_buffer_.push(pAgent->GetAllMapPoints());
    locker_point.unlock();
    point_cv_.notify_one();

    //std::cout<<"vor pose: "<<std::endl; // Debug

    PubPose();
    //std::cout<<" after pose , before cloud"<<std::endl;

    PubPointCloud();

    //std::cout<<" after cloud, before image"<<std::endl;

    PubImage();

    //std::cout<<" after image"<<std::endl;
    
    //* An example of what can be done after the pose w.r.t camera coordinate frame is computed by ORB SLAM3
    //Sophus::SE3f Twc = Tcw.inverse(); //* Pose with respect to global image coordinate, reserved for future use

}

sensor_msgs::msg::PointCloud2 MonocularMode::MapPointsToPointCloud(
        std::vector<ORB_SLAM3::MapPoint*> map_points) {

    sensor_msgs::msg::PointCloud2 cloud;

    Eigen::Matrix3f Rcw;
    Rcw << 0, 1, 0,
          -1, 0, 0,
           0, 0, 1;

    Eigen::Vector3f point;

    const int num_channels = 3; // x y z

    cloud.header.stamp = this->get_clock()->now();;
    cloud.header.frame_id = "map";
    cloud.height = 1;
    cloud.width = map_points.size();  //点的个数
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float); //一个点占用存储空间
    cloud.row_step = cloud.point_step * cloud.width; //整个点云占用存储空间
    cloud.fields.resize(num_channels);

    std::string channel_id[] = { "x", "y", "z"};
    for (int i = 0; i < num_channels; i++) {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    float data_array[num_channels];
    for (unsigned int i = 0; i < cloud.width; i++) {
        if (map_points.at(i)->nObs >= 2) {
            point = Rcw * map_points.at(i)->GetWorldPos();

            data_array[0] = (float)point(0);
            data_array[1] = (float)point(1);
            data_array[2] = (float)point(2);

            memcpy(cloud_data_ptr + (i * cloud.point_step),
                   data_array, num_channels * sizeof(float));
        }
    }
    return cloud;
}
tf2::Transform MonocularMode::TransformFromMat (cv::Mat position_mat) {
    cv::Mat rotation(3, 3, CV_32F);
    cv::Mat translation(3, 1, CV_32F);

    rotation = position_mat.rowRange(0, 3).colRange(0, 3);
    translation = position_mat.rowRange(0, 3).col(3);


    tf2::Matrix3x3 tf_camera_rotation (rotation.at<float> (0, 0),
                                       rotation.at<float> (0, 1),
                                       rotation.at<float> (0, 2),
                                       rotation.at<float> (1, 0),
                                       rotation.at<float> (1, 1),
                                       rotation.at<float> (1, 2),
                                       rotation.at<float> (2, 0),
                                       rotation.at<float> (2, 1),
                                       rotation.at<float> (2, 2));

    tf2::Vector3 tf_camera_translation (translation.at<float> (0),
                                        translation.at<float> (1),
                                        translation.at<float> (2));

    //Coordinate transformation matrix from orb coordinate system to ros coordinate system
    const tf2::Matrix3x3 tf_orb_to_ros (0, 1, 0,
                                       -1, 0, 0,
                                        0, 0, 1);

    //Transform from orb coordinate system to ros coordinate system on camera coordinates
    tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    //Inverse matrix
    tf_camera_rotation = tf_camera_rotation.transpose();
    tf_camera_translation = -(tf_camera_rotation * tf_camera_translation);

    //Transform from orb coordinate system to ros coordinate system on map coordinates
    tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    return tf2::Transform (tf_camera_rotation, tf_camera_translation);
}
void MonocularMode::PubPose() {
    Eigen::Matrix4f Tcw_Matrix;
    cv::Mat Tcw1;
    geometry_msgs::msg::TransformStamped tf_msg;
    geometry_msgs::msg::PoseStamped pose_msg;

    tf2_ros::TransformBroadcaster tf_broadcaster(this);

    //while (rclcpp::ok()) {
         
        std::unique_lock<std::mutex> locker_pose(pose_mutex_);
        while(pose_buffer_.empty())
            pose_cv_.wait(locker_pose);
        Tcw_Matrix = pose_buffer_.front().matrix();
        pose_buffer_.pop();
        locker_pose.unlock();

        cv::eigen2cv(Tcw_Matrix, Tcw1);
        tf2::Transform tf_transform = TransformFromMat(Tcw1);

        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "camera_link";
        tf_msg.header.stamp = this->get_clock()->now();
        tf_msg.transform = tf2::toMsg(tf_transform);

        tf_broadcaster.sendTransform(tf_msg);

        pose_msg.pose.position.x = tf_transform.getOrigin().getX();
        pose_msg.pose.position.y = tf_transform.getOrigin().getY();
        pose_msg.pose.position.z = tf_transform.getOrigin().getZ();

        pose_msg.pose.orientation.x = tf_transform.getRotation().getX();
        pose_msg.pose.orientation.y = tf_transform.getRotation().getY();
        pose_msg.pose.orientation.z = tf_transform.getRotation().getZ();
        pose_msg.pose.orientation.w = tf_transform.getRotation().getW();

        path_.header = tf_msg.header;
        path_.poses.push_back(pose_msg);
        path_publisher_->publish(path_);
    //}
}

void MonocularMode::PubImage() {
    sensor_msgs::msg::Image img_msg;
    cv_bridge::CvImage img_bridge;
    cv::Mat toshow;
    std_msgs::msg::Header header;
    bool received_image;
    header.frame_id = "camera_link";
    //while (rclcpp::ok()) {

        
       
        std::unique_lock<std::mutex> locker_image(image_mutex_);
        while (image_buffer_.empty())
            pose_cv_.wait(locker_image);
        received_image = image_buffer_.front();
        image_buffer_.pop();
        locker_image.unlock();
       

        if(received_image) {
            toshow = pAgent->GetmpFrameDrawer()->DrawFrame(1.0f);
            header.stamp = this->now();
            img_bridge = cv_bridge::CvImage(header, "bgr8", toshow);
            img_bridge.toImageMsg(img_msg);
            frame_publisher_->publish(img_msg);
        }
    //}
}

void MonocularMode::PubPointCloud(){
    std::vector<ORB_SLAM3::MapPoint *> orb_point;

    //while (rclcpp::ok()) {
        std::unique_lock<std::mutex> locker_point(point_mutex_);
        //orb_point = pAgent->GetAllMapPoints();

            while (point_buffer_.empty())
                point_cv_.wait(locker_point);

        orb_point = point_buffer_.front();
        point_buffer_.pop();
        locker_point.unlock();
        
        sensor_msgs::msg::PointCloud2 cloud = MapPointsToPointCloud(orb_point);
        pointcloud2_publisher_->publish(cloud);
    //}
}

