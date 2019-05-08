#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <geometry_msgs/Point.h>
#include <underwater_path_planning/Path.h>

int x_offset = 4;
int y_offset = 4;
int flag = 1;
using namespace std;

std::vector<int> robotY;
void planner_callback(const underwater_path_planning::Path::ConstPtr& msg){
    cout << " Hello" << endl;
}

void make_robot(ros::Publisher publisher){
    flag = flag * -1;
    visualization_msgs::Marker robot;
    auto shape = visualization_msgs::Marker::SPHERE;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    robot.header.frame_id = "/my_frame";
    robot.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    robot.ns = "robot";
    robot.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    robot.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    robot.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    if(flag == 1) robot.pose.position.x = 0;
    if(flag == -1) robot.pose.position.x = 1;
    robot.pose.position.y = 0;
    robot.pose.position.z = .5;

    // Set the color -- be sure to set alpha to something non-zero!
    robot.color.r = 0.0f;
    robot.color.g = 1.0f;
    robot.color.b = 0.0f;
    robot.color.a = 1.0;
    robot.scale.x = 1;
    robot.scale.y = 1;
    robot.scale.z = 1;

    robot.lifetime = ros::Duration();

    publisher.publish(robot);
    return;
}

void make_obstacles(ros::Publisher publisher){
    vector<int> x_obstacle = {1,5,4,5,2,7,2,4,3,8,6};
    vector<int> y_obstacle = {4,6,5,2,5,4,6,5,4,8,5};
    visualization_msgs::Marker obstacle;
    auto shape = visualization_msgs::Marker::CUBE_LIST;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    obstacle.header.frame_id = "/my_frame";
    obstacle.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    obstacle.ns = "obstacles";
    obstacle.id = 0;
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    obstacle.type = shape;
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    obstacle.action = visualization_msgs::Marker::ADD;
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    for (int i = 0; i <= 10; i++){
        geometry_msgs::Point temp;
        temp.x = x_obstacle[i];
        temp.y = y_obstacle[i];
        temp.z = 0.5;
        obstacle.points.push_back(temp);
    }

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    obstacle.scale.x = 1;
    obstacle.scale.y = 1;
    obstacle.scale.z = 1;

    // Set the color -- be sure to set alpha to something non-zero!
    obstacle.color.r = 1.0f;
    obstacle.color.g = 0.0f;
    obstacle.color.b = 0.0f;
    obstacle.color.a = 1.0;
    obstacle.lifetime = ros::Duration();

    // Publish the marker
    publisher.publish(obstacle);
    return;
}
int main( int argc, char** argv ) {
    ros::init(argc, argv, "shape_publisher");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher robot_pub = n.advertise<visualization_msgs::Marker>("robot_marker", 1);
    ros::Publisher obstacle_pub = n.advertise<visualization_msgs::Marker>("obstacle_markers", 1);
    // ros::Subscriber path_sub = n.subscribe<underwater_path_planning::Path>("planner_chatter",1000,planner_callback);
    make_robot(robot_pub);
    make_obstacles(obstacle_pub);

    int i = 0;
    while (ros::ok()) {
        make_robot(robot_pub);
        make_obstacles(obstacle_pub);
        ros::spinOnce;
        r.sleep();
    }
  return 1;
}
