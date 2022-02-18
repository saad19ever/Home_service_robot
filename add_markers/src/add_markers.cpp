#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cstdlib>

double pose[3];
double P_error[3] ;
double D_error[3]={1,1};
double Drop_off[3];
double Pick_up[3];

void AMCLCallback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
       pose[0] = msg->pose.pose.position.x;
       pose[1] = msg->pose.pose.position.y; 
       pose[2] = msg->pose.pose.orientation.w;           
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
   ros::Subscriber amcl_sub = n.subscribe("amcl_pose",10,AMCLCallback);
  
   
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
   //execute every cycle to ensure the subscriber receives the message
    ros::spinOnce();
   
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.
    marker.type = shape;


    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

     // Set the marker action
    marker.action = visualization_msgs::Marker::ADD;
    
    //get pick up location .
          n.getParam("/Pick_up/x", Pick_up[0]);
          n.getParam("/Pick_up/y", Pick_up[1]);
          n.getParam("/Pick_up/w", Pick_up[2]);
          
 // Set the pose of the marker
    marker.pose.position.x = Pick_up[0];
    marker.pose.position.y = Pick_up[1];
    marker.pose.position.z = 0.1;
    marker.pose.orientation.w = Pick_up[2];
    
     //publish the marker
    marker_pub.publish(marker);
        
     
    //calculate robot pose relative Pick up location error
    for (int i = 0 ; i < 3 ; i++ ) {
         P_error[i]= abs(Pick_up[i]-pose[i]);
    }
   
    //wait until robot reaches Pick up location
   if (P_error[0] <= 0.03 && P_error[1] <= 0.03 ){
   
   ROS_INFO("Robot successfully moved to the pick up location");
   
     //simulate pick up by deleting marker and waiting 5 seconds
   marker.action = visualization_msgs::Marker::DELETE;
   ros::Duration(1).sleep();
   marker_pub.publish(marker);
   ros::Duration(5.0).sleep();
  
   // Set the marker action again
   marker.action = visualization_msgs::Marker::ADD;
  
     //get drop off location .
          n.getParam("/Drop_off/x", Drop_off[0]);
          n.getParam("/Drop_off/y", Drop_off[1]);
          n.getParam("/Drop_off/w", Drop_off[2]);
          
 // Set the pose of the marker
    marker.pose.position.x = Drop_off[0];
    marker.pose.position.y = Drop_off[1];
    marker.pose.position.z = 0.1;
    marker.pose.orientation.w = Drop_off[2];
    
      
     while (!(D_error[0] <= 0.03) || !(D_error [1] <= 0.03))
     {  
        ROS_INFO("heading to drop off Location");
      
        //calculate robot pose relative drop off location error
        for (int i = 0 ; i < 3 ; i++ ) {
         D_error[i]=abs(Drop_off[i]-pose[i]);
          } 
           ros::spinOnce();
           ros::Duration(0.5).sleep();
     }
    //publish marker
    ROS_INFO("Robot successfully moved to the drop off Location");
    marker_pub.publish(marker);
    ros::Duration(200).sleep(); 
    }
    
     
    r.sleep();
  }
}
