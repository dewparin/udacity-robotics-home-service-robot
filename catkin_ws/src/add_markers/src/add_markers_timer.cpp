#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


int main( int argc, char** argv ){
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Time start_time;
    bool has_subscriber = false;

    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    bool marker_at_pickup = true;
    bool marker_at_dropoff = false;
    double pickup_x = 0.55439;
    double pickup_y = 2.7831;
    double dropoff_x = -1.83;
    double dropoff_y = -5.42;

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
            has_subscriber = false;
            sleep(1);
        }
        if (!has_subscriber) {
            has_subscriber = true;
            start_time = ros::Time::now();
            marker_at_pickup = true;
            marker_at_dropoff = false;
        }

        if (marker_at_pickup) {
            marker.pose.position.x = pickup_x;
            marker.pose.position.y = pickup_y;
            marker.action = visualization_msgs::Marker::ADD;
            ROS_INFO_ONCE("The marker is at the pickup zone");
            if ((ros::Time::now() - start_time).toSec() >= 5) {
                marker_at_pickup = false;
            }
        } else if (!marker_at_dropoff) {
            marker.action = visualization_msgs::Marker::DELETE;
            ROS_INFO_ONCE("The marker is hidden");
            if ((ros::Time::now() - start_time).toSec() >= 10) {
                marker_at_dropoff = true;
            }
        } else {
            marker.pose.position.x = dropoff_x;
            marker.pose.position.y = dropoff_y;
            marker.action = visualization_msgs::Marker::ADD;
            ROS_INFO_ONCE("The marker is at the drop-off zone");
        }

        marker.lifetime = ros::Duration();
        marker_pub.publish(marker);
        r.sleep();
    }
}