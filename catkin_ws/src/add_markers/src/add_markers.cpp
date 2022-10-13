#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


int main( int argc, char** argv ){
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    bool robot_at_pickup = false;
    bool robot_at_dropoff = false;
    double pickup_x = 0.0;
    double pickup_y = 0.0;
    double dropoff_x = 0.0;
    double dropoff_y = 0.0;

    while(ros::ok()) { 
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "add_markers";
        marker.id = 0;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = shape;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        //waiting for subscriber 
        while (marker_pub.getNumSubscribers() < 1){
            if (!ros::ok()){
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }

        n.getParam("robot_at_pickup", robot_at_pickup);
        n.getParam("robot_at_dropoff", robot_at_dropoff);
        if (!robot_at_pickup) {
            n.getParam("pickup_x", pickup_x);
            n.getParam("pickup_y", pickup_y);
            marker.pose.position.x = pickup_x;
            marker.pose.position.y = pickup_y;
            marker.action = visualization_msgs::Marker::ADD;
            ROS_INFO("The marker is at the pickup zone");
        } else if (!robot_at_dropoff) {
            marker.action = visualization_msgs::Marker::DELETE;
            ROS_INFO("The marker is in transit");
        } else {
            n.getParam("dropoff_x", dropoff_x);
            n.getParam("dropoff_y", dropoff_y);
            marker.pose.position.x = dropoff_x;
            marker.pose.position.y = dropoff_y;
            marker.action = visualization_msgs::Marker::ADD;
            ROS_INFO("The marker is at the drop-off zone");
        }

        marker.lifetime = ros::Duration();
        marker_pub.publish(marker);
        r.sleep();
    }
}