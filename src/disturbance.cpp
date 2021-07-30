#include "disturbance.h"

/* This Node applies wrench on a slected link of the robot by calling gazebo/ApplyBodyWrench service.
*/

disturbance::disturbance() //Constructor
{
    sub_clk = nh_.subscribe<rosgraph_msgs::Clock>("/clock",10,&disturbance::clk_callback,this);
    client = nh_.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench"); // service client of gazebo service
}

void disturbance::clk_callback(const rosgraph_msgs::Clock::ConstPtr &msg){

    my_time = msg->clock;

    // Wrench in body frame
    force.x = 10;
    force.y = 0;
    force.z = 0.0;
    moment.x = 0.0;
    moment.y = 0.0;
    moment.z = 0.0;

    try
    { 
        transformStamped = tfBuffer.lookupTransform("map", "base_link" , ros::Time(0)); // Transform from base_link to map frame
    }
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("%s",ex.what());
    }
    
    tf2::doTransform(force, force, transformStamped); // Transform from base_link to map
    tf2::doTransform(moment, moment, transformStamped); // Transform from base_link to map
    
    igor_wrench.force.x = force.x; // Force in newtons
    igor_wrench.force.y = force.y;
    igor_wrench.force.z = force.z;
    igor_wrench.torque.x = 0*moment.x; // Moment in Nm
    igor_wrench.torque.y = 0*moment.y;
    igor_wrench.torque.z = 0*moment.z;
    srv.request.body_name = igor_body_name;
    srv.request.wrench = igor_wrench;
    srv.request.duration = ros::Duration(1);

    if (my_time.toSec()==10 || my_time.toSec()==50){ // To call the rosservice at the 10th sec
        if(!run){
            ROS_INFO("Calling Apply_Body_Wrench Service @ %f secs", my_time.toSec() );
            std::cout << "Wrench vector: " << igor_wrench << std::endl;
            client.call(srv); // Call the service
            //run = true; // Run only once
            if(srv.response.success){
                ROS_INFO("Wrench applied successfully");
            } 
        }

    }

} // end of clk_callback

disturbance::~disturbance(){

}

int main(int argc, char **argv){


ros::init(argc, argv, "disturber");

disturbance my_disturbance_node; // creating the disturbance object

ros::spin(); // only need for subscriber to call its callback function (Does not need for Publisher).

return 0;

} // end of main