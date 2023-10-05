#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_broadcaster.h>


//callback funtion for the gazebo_msgs/link_states subscriber
ros::Publisher path_pub;
//two addresses are created in order to update the stored amounts inside them through callback function.
tf::TransformBroadcaster *broadcaster;
tf::Transform *transform;
void Callback(const gazebo_msgs::ModelStates& data) {
    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;
    path.header.stamp=ros::Time::now();
    path.header.frame_id="map";
    pose.header.stamp=ros::Time::now();
    pose.header.frame_id="base_footprint";

    pose.pose.position.x = data.pose[1].position.x;
    pose.pose.position.y = data.pose[1].position.y;
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            pose.pose.orientation.w = 1;
            path.poses.push_back(pose);
            path_pub.publish(path);

            broadcaster = new tf::TransformBroadcaster;
                    transform = new tf::Transform;
                    //set origination of the transform fixed
                    transform->setOrigin( tf::Vector3(0, 0, 0) );
                    tf::Quaternion q;
                    // set the quaternion using Euler angles
                    q.setEulerZYX(tf::getYaw(data.pose[1].orientation), 0, 0);
                    transform->setRotation( q );
                    *transform = transform->inverse();
                    //pass the pointer to show the real-time stored amount in that address(includes the inverse of the tf_transform)
                    broadcaster->sendTransform(tf::StampedTransform(*transform, ros::Time::now(), "base_footprint", "map"));




}

//node initialization in order to subscribe position from an advertised topic named '/gazebo/link_states' and store needed variables to publish as a navigation messages which can be visualized by Rviz to see the trajectory real-time.
int main (int argc, char **argv)
{

        ros::init (argc, argv, "by_path");
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("/gazebo/model_states", 1, Callback);
         path_pub = n.advertise<nav_msgs::Path>("trajectory",1, true);
      ros::Rate loop_rate(40);
        while (ros::ok())
           {
               loop_rate.sleep(); //Maintain the loop rate
               ros::spinOnce();   //Check for new messages
           }


        return 0;
}

