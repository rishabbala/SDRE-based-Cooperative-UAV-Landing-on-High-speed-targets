/* Author : Vishvender Malik
Email : vishvenderm@iiitd.ac.in
File : monitor_wind.cpp
*/

#include <boost/python.hpp>
//#include </home/vishvender17/monitor_ws/src/pkg_monitor_library/include/pkg_monitor_library/interface_monitor_library_wind.h>
#include <string>
#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <ctime>
#include <pkg_monitor_library/TargetInfo.h>
#include <pkg_monitor_library/Contour.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point32.h>

using namespace boost::python;

//--------------------Global variables--------------------------------
std::string topic_desired_airspeed;
std::string topic_blobDataFlags;
std::string topic_guidance_velocity;

const int array_actual_velocity_size = 3, array_wind_velocity_size = 3, array_guidance_velocity_size = 3,
          array_total_irl_vel_size = 3, size_vel_difference = 3, size_covariance = 36, array_new_airspeed_size = 3;
double array_desired_airspeed[array_actual_velocity_size] = {0}, array_windspeed[array_wind_velocity_size] = {0},
       array_groundspeed_guidance[array_guidance_velocity_size] = {0}, array_actual_groundspeed[array_total_irl_vel_size] = {0},
       array_vel_difference[size_vel_difference] = {0}, array_new_airspeed[array_new_airspeed_size] = {0};
int flag_target_detected, flag_target_tracking;
float altitude = 0.0, array_covariance[size_covariance], sign_groundspeed_guidance_vel_x, sign_groundspeed_guidance_vel_y, sign_groundspeed_guidance_vel_z,
      sign_actual_groundspeed_vel_x, sign_actual_groundspeed_vel_y, sign_actual_groundspeed_vel_z, max_possible_vel_in_x, max_possible_vel_in_y,
      max_possible_vel_in_z;
float target_center_x, target_center_y;

geometry_msgs::TwistStamped command; // final command message to be published

// publishers and subscribers
ros::Publisher pub_landing_target_info;
ros::Publisher pub_corrected_airspeed;
ros::Subscriber sub_groundspeed_guidance;
ros::Subscriber sub_desired_airspeed;
ros::Subscriber sub_receive_altitude;
ros::Subscriber sub_get_windspeed;
ros::Subscriber sub_vision_landing_target_info;
//------------------------------------------------------------------------------------------>
//<--------------------------Function declarations------------------------------------->

void receiveDesiredAirspeed(const geometry_msgs::TwistStamped::ConstPtr &data);
void receiveGroundspeedGuidance(const geometry_msgs::TwistStamped::ConstPtr &data);
void receiveAltitude(const std_msgs::Float64::ConstPtr &data);
void getWindspeed(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &data);
void vision_landing_target_info(const pkg_monitor_library::TargetInfo::ConstPtr &info);
void calculate_prediction();
void publish_final_command();

    //<------------------------------------------------------------------------------------>

// base class, acting as an interface for our API
/*
class monitor_interface{
    public: 
    // function to be implemented in dependent classes
    virtual void monitor_start() = 0;
};
*/

// class dependent on base class monitor_interface
/*class monitor_wind: public monitor_interface{    
   private:
    // publishers and subscribers
    ros::Publisher pub_landing_target_info;
    ros::Publisher pub_corrected_airspeed;
    ros::Subscriber sub_groundspeed_guidance;
    ros::Subscriber sub_desired_airspeed;
    ros::Subscriber sub_receive_altitude;
    ros::Subscriber sub_get_windspeed;
    ros::Subscriber sub_vision_landing_target_info;

    //<--------------------------Function declarations------------------------------------->

    void receiveDesiredAirspeed(const geometry_msgs::TwistStamped::ConstPtr &data);
    void receiveGroundspeedGuidance(const geometry_msgs::TwistStamped::ConstPtr &data);
    void receiveAltitude(const std_msgs::Float64::ConstPtr &data);
    void getWindspeed(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &data);
    void vision_landing_target_info(const pkg_monitor_library::TargetInfo::ConstPtr &info);
    void calculate_prediction();
    void publish_final_command();

    //<------------------------------------------------------------------------------------>
*/

// failsafe for if this header is used more than once in
// the same file (will compile it only once)
#ifndef interface_monitor_library_wind
#define interface_monitor_library_wind
class monitor_wind{        
    public:
    monitor_wind();
    ~ monitor_wind();// so that user could stop monitor at will
    void set_topic_guidance_velocity(std::string topic);
    void set_topic_blobDataFlags(std::string topic);
    void set_topic_desired_airspeed(std::string topic);// optional
    void monitor_start();
};
#endif

  //  public:
monitor_wind::monitor_wind(){
    ROS_INFO("\n\nWind Monitor is created and initialized.\n\n");
}

monitor_wind::~ monitor_wind(){
    ROS_INFO("\n\nWind Monitor is stopped and destructed.\n\n");
}

void monitor_wind::set_topic_desired_airspeed(std::string topic)
{
    topic_desired_airspeed = topic;
}

void monitor_wind::set_topic_blobDataFlags(std::string topic)
{
    topic_blobDataFlags = topic;
}

void monitor_wind::set_topic_guidance_velocity(std::string topic)
{
    topic_guidance_velocity = topic;
}

void monitor_wind::monitor_start()
{
    ROS_INFO("\n\n----------------Welcome-----------------\n\n");

    // initialize ros node with a node name
    //ros::init(argc, argv, "node_monitor");

    // create a nodehandle to enable interaction with ros commands, always just after ros::init
    ros::NodeHandle nodeHandle;

    // create publisher and subscriber objects

    // final publisher to application // topic should just be cmd_vel
    pub_corrected_airspeed = nodeHandle.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1000); // should be cmd_vel?

    // subscriber to intercept velocity commands from guidance controller
    //sub_groundspeed_guidance = nodeHandle.subscribe("/mavros/setpoint_velocity/cmd_vel_new", 1000, receiveGroundspeedGuidance);
    sub_groundspeed_guidance = nodeHandle.subscribe(topic_guidance_velocity, 1000, receiveGroundspeedGuidance);
    // subscriber to receive velocity commands from the topic itself
    sub_desired_airspeed = nodeHandle.subscribe(topic_desired_airspeed, 1000, receiveDesiredAirspeed);
    // subscriber to receive altitude data from the topic itself
    // but we aren't getting it from the application yet
    sub_receive_altitude = nodeHandle.subscribe("/mavros/global_position/rel_alt", 1000, receiveAltitude);
    // subscriber to get wind data information
    sub_get_windspeed = nodeHandle.subscribe("/mavros/wind_estimation", 1000, getWindspeed);
    // subscriber to receive landing target info from the vision controller
    sub_vision_landing_target_info = nodeHandle.subscribe(topic_blobDataFlags, 1000, vision_landing_target_info);
    // sub_vision_landing_target_info = nodeHandle.subscribe("/landing_target_info_new", 10, vision_landing_target_info); // original

    // Use current time as seed for random generator
    srand(time(0));

    //run loop at (10) Hz (always in decimal and faster than what is published through guidance controller)
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce(); // if we have subscribers in our node, but always keep for good measure

        publish_final_command(); // keep calling this function

        // sleep for appropriate time to hit mark of (10) Hz
        loop_rate.sleep();
    } // end of while loop
} // end of monitor_start()

//<------------------------------------------Function definitions------------------------------------------------------>

// function to receive Desired airspeed (published on to the topic) from the topic itself
void receiveDesiredAirspeed(const geometry_msgs::TwistStamped::ConstPtr &data)
{
    array_desired_airspeed[0] = data->twist.linear.x;
    array_desired_airspeed[1] = data->twist.linear.y;
    array_desired_airspeed[2] = data->twist.linear.z;
    ROS_INFO("Data received from topic \"/mavros/local_position/velocity\".");
}

// function to receive altitude information (published on to the topic) from the topic itself
void receiveAltitude(const std_msgs::Float64::ConstPtr &data)
{
    altitude = data->data;
    ROS_INFO("Data received from topic \"/mavros/global_position/rel_alt\".");
}

// function to receive wind related information from the topic itself
void getWindspeed(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &data)
{                                                    // for simulation purpose, gen random values in range -6 to +6
    array_windspeed[0] = data->twist.twist.linear.x; //rand() % 13 + (-6);
    array_windspeed[1] = data->twist.twist.linear.y; //rand() % 13 + (-6);
    array_windspeed[2] = data->twist.twist.linear.z; //rand() % 13 + (-6);
    ROS_INFO("Data received from topic \"/mavros/wind_estimation\".");
    // but won't show any data since none is being published on it yet so it doesn't show up in callback

    // put covariance values in array or possibly in a 6 x 6 Eigen matrix for further evaluation
    for (int index = 0; index < 36; index++)
        array_covariance[index] = data->twist.covariance[index]; // figure out it's use
}

// function to receive landing target info from vision controller
void vision_landing_target_info(const pkg_monitor_library::TargetInfo::ConstPtr &info)
{
    flag_target_detected = info->detected;
    flag_target_tracking = info->tracking;
    target_center_x = info->contour.center.x; // distance x from centre of target
    target_center_y = info->contour.center.y; // distance y from centre of target
    ROS_INFO("Data received from topic \"/landing_target_info\".");
}

// function to receive velocity commands published by guidance controller for comparison
// and also copy their respective signs for further use
void receiveGroundspeedGuidance(const geometry_msgs::TwistStamped::ConstPtr &data)
{
    // for simulation purpose, gen random values in range -5 to +5
    array_groundspeed_guidance[0] = data->twist.linear.x; //rand() % 11 + (-5);
    sign_groundspeed_guidance_vel_x = copysign(1, array_groundspeed_guidance[0]);
    array_groundspeed_guidance[1] = data->twist.linear.y; //rand() % 11 + (-5);
    sign_groundspeed_guidance_vel_y = copysign(1, array_groundspeed_guidance[1]);
    array_groundspeed_guidance[2] = data->twist.linear.z; //rand() % 11 + (-5);
    sign_groundspeed_guidance_vel_z = copysign(1, array_groundspeed_guidance[2]);
    ROS_INFO("Data received from topic \"/mavros/setpoint_velocity/cmd_vel_new\".");
}

// function to calculate required predictions and populate "command" message
void calculate_prediction()
{

    // make calculations

    // APPROACH
    // we have velocity parameters from vel topic and wind topic
    // calculate relative velocity in all three directions
    // needs to intercept messages from pubCommand publisher from guidance node to compare
    // if values are within +1 and -1, let original messages passthrough
    // if values are bigger, make corrections and publish new values in command message
    // as long as the UAV is moving in the required direction, we can pass relative or guidance velocity accordingly
    // if direction changes (from +ve to -ve and vice versa), then calculate effective new velocity and publish them
    // also check if new effective velocities <= max possible velocity of the vehicle at that moment in time (using contour.x, contour.y)
    // if calculated effective velocity > max possible velocity, publish only max possible velocity

    // Edit this logic
    max_possible_vel_in_x = target_center_x / float(20); // publishing timer for topic /landing_target_info_new in application
    max_possible_vel_in_y = target_center_y / float(20);

    for (int index = 0; index < 3; index++)
    {
        // Wind Triangle equation
        array_actual_groundspeed[index] = array_desired_airspeed[index] + array_windspeed[index];
        array_vel_difference[index] = array_groundspeed_guidance[index] - array_actual_groundspeed[index];
        /*  
        if((array_desired_airspeed[index] > 0 && array_windspeed[index] < 0) || 
            (array_desired_airspeed[index] < 0 && array_windspeed[index] > 0)){

                array_actual_groundspeed[index] = array_desired_airspeed[index] + array_windspeed[index];
                array_vel_difference[index] = array_groundspeed_guidance[index] - array_actual_groundspeed[index]; 
        }
        else if((array_desired_airspeed[index] >= 0 && array_windspeed[index] >= 0) || 
            (array_desired_airspeed[index] <= 0 && array_windspeed[index] <= 0)){

                array_actual_groundspeed[index] = array_desired_airspeed[index] - array_windspeed[index];
                array_vel_difference[index] = array_groundspeed_guidance[index] - array_actual_groundspeed[index]; 
        } 
        */
    }

    // copy signs of irl velocity parameters for comparison
    sign_actual_groundspeed_vel_x = copysign(1, array_actual_groundspeed[0]);
    sign_actual_groundspeed_vel_y = copysign(1, array_actual_groundspeed[1]);
    sign_actual_groundspeed_vel_z = copysign(1, array_actual_groundspeed[2]);

    // assuming target is already in sight and is being tracked
    if (flag_target_detected && flag_target_tracking)
    {

        for (int index = 0; index < 3; index++)
        {
            // array_new_airspeed = new_groundspeed - windspeed (Wind Triangle equation)
            array_new_airspeed[index] = array_desired_airspeed[index] - array_windspeed[index];
        }

        if (array_new_airspeed[0] == array_groundspeed_guidance[0])
        {
            command.twist.linear.x = array_groundspeed_guidance[0];
        }
        else
        {
            if (abs(array_new_airspeed[0]) <= abs(max_possible_vel_in_x))
            {
                command.twist.linear.x = array_new_airspeed[0];
            }
            else
            {
                command.twist.linear.x = max_possible_vel_in_x;
            }
        }

        if (array_new_airspeed[1] == array_groundspeed_guidance[1])
        {
            command.twist.linear.y = array_groundspeed_guidance[1];
        }
        else
        {
            if (abs(array_new_airspeed[1]) <= abs(max_possible_vel_in_y))
            {
                command.twist.linear.y = array_new_airspeed[1];
            }
            else
            {
                command.twist.linear.y = max_possible_vel_in_y;
            }
        }

        if (array_new_airspeed[2] == array_groundspeed_guidance[2])
        {
            command.twist.linear.z = array_groundspeed_guidance[2];
        }
        else
        {
            // Because there's no max velocity calculated for parameter y
            command.twist.linear.z = array_new_airspeed[2];
            /* 
            if(abs(array_new_airspeed[2]) <= abs(max_possible_vel_in_z)){    
                    command.twist.linear.z = array_new_airspeed[2];
                }
                else{
                    command.twist.linear.z = max_possible_vel_in_z;
                }
            */
        }

        /*
        if(sign_groundspeed_guidance_vel_x == sign_actual_groundspeed_vel_x){

            if(array_vel_difference[0] <= 1 && array_vel_difference[0] >= -1){
                command.twist.linear.x = array_desired_airspeed[0];
            }
            else{

                if(abs(- (array_actual_groundspeed[0] - array_groundspeed_guidance[0])) <= abs(max_possible_vel_in_x)){    
                    command.twist.linear.x = - (array_actual_groundspeed[0] - array_groundspeed_guidance[0]);
                }
                else{
                    command.twist.linear.x = max_possible_vel_in_x;
                }              
            }
        }
        else{
            
            if(abs(copysign((array_actual_groundspeed[0] - array_groundspeed_guidance[0]), array_groundspeed_guidance[0])) <= abs(max_possible_vel_in_x)){
                command.twist.linear.x = copysign((array_actual_groundspeed[0] - array_groundspeed_guidance[0]), array_groundspeed_guidance[0]);
            }
            else{
                command.twist.linear.x = copysign(max_possible_vel_in_x, array_groundspeed_guidance[0]);
            }           
        }
            
        if(sign_groundspeed_guidance_vel_y == sign_actual_groundspeed_vel_y){

            if(array_vel_difference[1] <= 1 && array_vel_difference[1] >= -1){
                command.twist.linear.y = array_desired_airspeed[1];
            }
            else{

                if(abs(- (array_actual_groundspeed[1] - array_groundspeed_guidance[1])) <= abs(max_possible_vel_in_y)){
                    command.twist.linear.y = - (array_actual_groundspeed[1] - array_groundspeed_guidance[1]);
                }
                else{
                    command.twist.linear.y = max_possible_vel_in_y;
                }   
            }
        }
        else{

            if(abs(copysign((array_actual_groundspeed[1] - array_groundspeed_guidance[1]), array_groundspeed_guidance[1])) <= abs(max_possible_vel_in_y)){
                command.twist.linear.y = copysign((array_actual_groundspeed[1] - array_groundspeed_guidance[1]), array_groundspeed_guidance[1]);
            }
            else{
                command.twist.linear.y = copysign(max_possible_vel_in_y, array_groundspeed_guidance[1]);
            }          
        }

        if(sign_groundspeed_guidance_vel_z == sign_actual_groundspeed_vel_z){

            if(array_vel_difference[2] <= 1 && array_vel_difference[2] >= -1){
                command.twist.linear.z = array_desired_airspeed[2];
            }
            else{
                command.twist.linear.z = - (array_actual_groundspeed[2] - array_groundspeed_guidance[2]);
            }
        }
        else{
            command.twist.linear.z = copysign((array_actual_groundspeed[2] - array_groundspeed_guidance[2]), array_groundspeed_guidance[2]);
        }
    */
    } // end of big if
    else
    {
        // either hold the position or return back
        command.twist.linear.x = 0;
        command.twist.linear.y = 0;
        command.twist.linear.z = 0;
    }
} // end of function calculate_prediction

// function to publish final command through the publisher via this monitor
void publish_final_command()
{

    ROS_INFO("\n\n------------------------------------Data received----------------------------------------\n\n");

    ROS_INFO("\n\nDesired airspeed received via topic for parameter x : %f \n"
            "Desired airspeed received via topic for parameter y : %f \n"
            "Desired airspeed received via topic for parameter z : %f \n",
            array_desired_airspeed[0], array_desired_airspeed[1],
            array_desired_airspeed[2]);

    ROS_INFO("\n\nWind speed received via topic for parameter x : %f \n"
            "Wind speed received via topic for parameter y : %f \n"
            "Wind speed received via topic for parameter z : %f \n",
            array_windspeed[0], array_windspeed[1],
            array_windspeed[2]);

    ROS_INFO("\n\nAltitude Info received from vision controller : %f\n", altitude);

    ROS_INFO("\n\nTarget distance received from vision controller for parameter x : %f \n"
            "Target distance received from vision controller for parameter y : %f \n",
            target_center_x, target_center_y);

    ROS_INFO("\n\nFlag value for target being detected : %d \n"
            "Flag value for target being tracked : %d \n",
            flag_target_detected, flag_target_tracking);

    ROS_INFO("\n\nGroundspeed received for comparison from guidance controller for parameter x : %f \n"
            "Groundspeed received for comparison from guidance controller for parameter y : %f \n"
            "Groundspeed received for comparison from guidance controller for parameter z : %f\n",
            array_groundspeed_guidance[0], array_groundspeed_guidance[1], array_groundspeed_guidance[2]);

    // make prediction at set frequency
    calculate_prediction();

    ROS_INFO("\n\nCorrected airspeed published by monitor for parameter x : %f \n"
            "Corrected airspeed published by monitor for parameter y : %f \n"
            "Corrected airspeed published by monitor for parameter z : %f\n\n",
            command.twist.linear.x, command.twist.linear.y, command.twist.linear.z);

    ROS_INFO("\n\n------------------------------------End of data block----------------------------------------\n\n");

    // finally, publish to the topic
    pub_corrected_airspeed.publish(command);
}
//}; // end of class monitor_wind


BOOST_PYTHON_MODULE(libpython_monitor_wind)
{
    class_<monitor_wind>("monitor_wind")// default constructor is exposed automatically
        .def("set_topic_guidance_velocity", &monitor_wind::set_topic_guidance_velocity)
        .def("set_topic_blobDataFlags", &monitor_wind::set_topic_blobDataFlags)
        .def("set_topic_desired_airspeed", &monitor_wind::set_topic_desired_airspeed)
        .def("monitor_start", &monitor_wind::monitor_start)
        //.def("~monitor_wind", &monitor_wind::~monitor_wind)
    ;
}