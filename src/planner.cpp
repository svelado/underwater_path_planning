#include <iostream>
#include <vector>
#include <assert.h>
#include <stdlib.h>
#include <time.h>
#include <cstdlib>  // For srand() and rand()
#include <math.h>
#include "ros/ros.h"
// #include "underwater_path_planning/Path.h"
#include "underwater_path_planning/PathPlan.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>


#include <sstream>



using namespace std;

class Node;
class Edge;
double straightEdgeCost = 1;
double diagonalEdgeCost = 1.4;
vector<int> obstacle_x = {1,5,4,5,2,7,2,4,3,8,6};
vector<int> obstacle_y = {4,6,5,2,5,4,6,5,4,8,5};
int xIndex = 0;
int yIndex = 1;
int paths = 0;
ros::Publisher path_pub;


class Node {
    private:
        int x;
        int y;
        bool closed;
        double heuristic;
        vector<Edge *> edges;
        bool obstacle;
        double gValue;
        vector<int> nextNode;
        vector<int> prevNode;
        double visitPenalty;

    public:
        Node(int x,int y,bool closed=false) {
            this->x = x;
            this->y = y;
            this->closed = false;
            this->obstacle = false;
            this->gValue = 100;
            this->visitPenalty = 0;
        }

        vector<int> getCoords(){
            vector<int> coords;
            coords.push_back(this->x);
            coords.push_back(this->y);
            return coords;
        }
        void visit(){
            this->visitPenalty += 5;
        }
        void close() {
            this->closed = true;
        }
        void open() {
            this->closed = false;
        }
        vector<Edge *> getEdges(){
                return this->edges;
        }

        void addEdge(Edge * newEdge){
            this->edges.push_back(newEdge);
            return;
        }
        bool isClosed(){
            return this->closed;
        }
        void makeObstacle(){
            this->closed = false;
            this->obstacle = true;
            return;
        }
        bool isObstacle(){
            return this->obstacle;
        }
        int getGValue(){
            return this->gValue;
        }
        void setGValue(int gValue){
            this->gValue = gValue;
            return;
        }
        //Heuristic is euclidean distance
        void setHeuristic(vector<int> endCoords) {
            int end_x = endCoords[0];
            int end_y = endCoords[1];
            this->heuristic = sqrt(pow(end_x - this->x,2) + pow(end_y - this->y,2));
            this->heuristic = this->heuristic + this->visitPenalty;
            return;
        }
        double getHeuristic(){
            return this->heuristic;
        }
        void setNextNode(vector<int> next) {
            this->nextNode = next;
        }
        void setPrevNode(vector<int> prev) {
            int x = prev[0];
            int y = prev[1];
            this->prevNode.push_back(x);
            this->prevNode.push_back(y);
            return;
        }
        vector<int> getNextNode(){
            return this->nextNode;
        }
        vector<int> getPrevNode(){
            return this->prevNode;
        }
        void clearNeighbors(){
            while(!this->nextNode.empty()){
                nextNode.pop_back();
            }
            while(!this->prevNode.empty()){
                this->prevNode.pop_back();
            }
        }

};

class Edge{
    private:
        double edgeCost;
        Node * start;
        Node * end;
    public:
        Edge(Node * start, Node * end){
            this->start = start;
            this->end = end;
            return;
        }

        void setCost(double cost) {
            this->edgeCost = cost;
            return;
        }

        Node * getStart() {
            return this->start;
        }

        Node * getEnd() {
            return this->end;
        }

        double getCost() {
            return this->edgeCost;
        }
};

class Graph {
    private:
        int x_size;
        int y_size;
        vector<vector<Node *>> nodes;

    public:
        Graph(int x_size,int y_size) {
            this->x_size = x_size;
            this->y_size = y_size;

            for (int x = 0; x < x_size; x++) {
                vector<Node*> newRow;
                for(int y = 0; y < y_size; y++ ) {
                    Node * newNode = new Node(x,y);
                    newRow.push_back(newNode);
                }
                this->nodes.push_back(newRow);
            }
            for (int x = 0; x < x_size; x++) {
                for(int y = 0; y < y_size; y++ ) {
                    Node * start = nodes[x][y];
                    for (int i = -1; i < 2; i++){
                        int temp_x = x + i;
                        for (int j = -1; j < 2 ; j++) {
                            int temp_y = y + j;
                            bool sameNode = temp_x == x && temp_y == y;
                            if (0 <= temp_x && temp_x < this->x_size && 0 <= temp_y && temp_y < this->y_size && !sameNode){
                                Node * nextNode = nodes[temp_x][temp_y];
                                Edge * newEdge = new Edge(start,nextNode);
                                if(i == 0 || j == 0) {
                                    newEdge->setCost(straightEdgeCost);
                                } else {
                                    newEdge->setCost(diagonalEdgeCost);
                                }
                                start->addEdge(newEdge);
                            }
                        }
                    }
                }
            }
        }

        void printGraph(){
            for (int row = y_size - 1; row >= 0; row--) {
                for(int col = 0; col < x_size; col++ ) {
                    Node * node = nodes[col][row];
                    vector<int> coords = node->getCoords();
                    cout << "(" << coords[0] << "," << coords[1] << ")" << " ";
                }
                cout << endl;
            }
        }

        Node * getNode(int x, int y){
            return this->nodes[x][y];
        }

        void printObstacles(){
            for (int row = y_size - 1; row >= 0; row--) {
                for(int col = 0; col < x_size; col++ ) {
                    Node * node = nodes[col][row];
                    if(node->isObstacle()) {
                        cout << "1 ";
                    } else {
                        cout << "0 ";
                    }
                }
                cout << endl;
            }
        }

        void calculateHeuristic(vector<int> goal){
            for (int row = 0; row < this->y_size ; row++) {
                for (int col = 0; col < this->x_size; col++) {
                    Node * node = this->getNode(col,row);
                    node->setHeuristic(goal);
                }
            }
        }
        void resetGraph(){
            for (int row = 0; row < this->y_size ; row++) {
                for (int col = 0; col < this->x_size; col++) {
                    Node * node = this->getNode(col,row);
                    node->setGValue(100);
                    node->open();
                    node->clearNeighbors();
                }
            }
        }
        void setObstacles(){
            for (int i = 0; i < 11; i++){
                int x_obs = obstacle_x[i];
                int y_obs = obstacle_y[i];
                Node * newObs = this->getNode(x_obs,y_obs);
                newObs->makeObstacle();
                assert(newObs->isObstacle());
            }
        }
};

void testNode() {
    Node * testNode = new Node(1,1);
    cout << "Testing Node Class.." << endl;
    auto coords = testNode->getCoords();
    assert(coords[0] == 1);
    assert(coords[1] == 1);

    cout << "Works!" << endl;

}

void testEdge(){
    // Test edge creation
    Node * start = new Node(0,0);
    Node * end = new Node(2,2);
    cout << "Testing Edge Class..." << endl;
    Edge * testEdge = new Edge(start,end);
    assert(testEdge->getStart() == start);
    assert(testEdge->getEnd() == end);
    testEdge->setCost(5);
    assert(testEdge->getCost() == 5);
    //Now adding the edge to the nodes
    start->addEdge(testEdge);
    cout << "Works!" << endl;
}

void testGraph() {
    cout << "Testing Graph Class..." << endl;
    Graph * testGraph = new Graph(100,100);
    cout << "Printing graph.." << endl;
    testGraph->printGraph();
    cout << "Getting a random node:" << endl;
    Node * testNode = testGraph->getNode(9,9);
    vector<Edge *> edges = testNode->getEdges();
    cout << "Checking Edges:" << endl;
    for (auto edge : edges) {
        Node * start = edge->getStart();
        Node * end = edge->getEnd();
        auto startCoords = start->getCoords();
        auto endCoords = end->getCoords();
        int x_start = startCoords[0];
        int y_start = startCoords[1];
        int x_end = endCoords[0];
        int y_end = endCoords[1];
        cout << "(" << x_start << "," << y_start << ")" << "-->" << "(" << x_end << "," << y_end << ")";
        cout << " Edge cost: " << edge->getCost() << endl;
    }

}

void testNodePointers(){
    cout << "Testing node pointers..." << endl;
    Graph * testGraph = new Graph(100,100);
    Node * testNode = testGraph->getNode(5,5);
    auto edges = testNode->getEdges();
    for (auto edge : edges) {
        Node * end = edge->getEnd();
        end->close();
    }
    Node * node44 = testGraph->getNode(4,4);
    Node * node66 = testGraph->getNode(6,6);
    assert(node44->isClosed() == true);
    assert(node66->isClosed() == true);
    cout << "Works!" << endl;
}

void testClasses(){
    testNode();
    testEdge();
    testGraph();
    testNodePointers();
}

void generateObstacles(){
    Graph * graph = new Graph(9,9);
    vector<int> x_obstacle = {-2,2,-3,1,-1,0,2,0,-3,2,-3};
    vector<int> y_obstacle = {-2,0,1,2,-1,2,1,-2,3,1,-3};
    //Add obstacles
    for (int i = 0; i <= 10; i++) {
        Node * obstacleNode = graph->getNode(x_obstacle[i] ,y_obstacle[i]);
        obstacleNode->makeObstacle();
        assert(obstacleNode->isObstacle());
    }
    graph->printObstacles();
}


int findSmallestF(vector<Node *> open){
    int minIndex = 0;
    double lowestF = 100;
    for (int index = 0; index < open.size();index++) {
        Node * node = open[index];
        double f = node->getGValue() + node->getHeuristic();
        if(f <= lowestF) {
            minIndex = index;
            lowestF = f;
        }
    }
    return minIndex;
}

void testSmallestF(){
    cout << "Testing smallestF..." << endl;
    Node * test1 = new Node(1,1);
    Node * test2 = new Node(1,2);
    Node * test3 = new Node(1,3);
    test1->setGValue(1);
    test2->setGValue(2);
    test3->setGValue(3);
    vector<Node *> testNodes;
    testNodes.push_back(test1);
    testNodes.push_back(test2);
    testNodes.push_back(test3);
    assert(findSmallestF(testNodes) == 0);
    cout << "Works!" << endl;
}

//An informed A * Search
vector<vector<int>> aSearch(vector<int> start,vector<int> goal, Graph * graph){
    //First set the heuristic with our given goal
    graph->calculateHeuristic(goal);
    vector<vector<int>> path;
    vector<Node *> open;
    Node * start_node = graph->getNode(start[0],start[1]);
    Node * goal_node = graph->getNode(goal[0],goal[1]);
    start_node->setGValue(0);
    open.push_back(start_node);
    Node * sPrime = NULL;
    Node * s = NULL;

    //Compute Path Function
    while(!goal_node->isClosed() && !open.empty()) {
        int minIndex = findSmallestF(open);
        s = open[minIndex];
        open.erase(open.begin() + minIndex);
        s->close();
        for (auto edge : s->getEdges()) {
            Node * sPrime = edge->getEnd();
            sPrime->setPrevNode(s->getCoords());
            if(!sPrime->isClosed() && !sPrime->isObstacle()) {
                if(sPrime->getGValue() > s->getGValue() + edge->getCost()) {
                    sPrime->setGValue(s->getGValue() + edge->getCost());
                    open.push_back(sPrime);
                }
            }

        }
    }

    if(open.empty()) {
        cout << "No solution!" << endl;
        return path;
    }
    Node * tracker = goal_node;
    vector<int> tracker_coords = goal_node->getCoords();
    path.push_back(tracker_coords);
    int sx = 100;
    int sy = 100;
    int sx_prev = 100;
    int sy_prev = 100;
    while(true){
        tracker->visit();
        vector<int> prev = tracker->getPrevNode();
        sx = prev[0];
        sy = prev[1];
        if(sx == start[0] && sy == start[1]) break;
        tracker = graph->getNode(sx,sy);
        path.insert(path.begin(),1,prev);
    }

    return path;

}

void printPath(vector<vector<int>> path){
    cout << "Path to be taken:" << endl;
    for (auto point : path) {
        cout << "(" << point[0] << "," << point[1] <<")" << endl;
    }
    cout << endl;

}

//Creates our master graph
Graph * graph = new Graph(9,9);

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

void make_path(vector<geometry_msgs::Point> pathPoints){
    paths++;
    visualization_msgs::Marker path;
    auto shape = visualization_msgs::Marker::LINE_STRIP;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    path.header.frame_id = "/my_frame";
    path.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    path.ns = "path";
    path.id = paths;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    path.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    path.action = visualization_msgs::Marker::ADD;



    // Set the color -- be sure to set alpha to something non-zero!
    path.color.r = 0.0f;
    path.color.g = 0.0f;
    path.color.b = 1.0f;
    path.color.a = 1.0;
    path.scale.x = .25;

    //Set points
    path.points = pathPoints;

    path.lifetime = ros::Duration();

    path_pub.publish(path);
    return;
}

bool plan(underwater_path_planning::PathPlan::Request &req,
        underwater_path_planning::PathPlan::Response &res){
    graph->setObstacles();
    //Unpack request
    int startX = req.startX;
    int startY = req.startY;
    int goalX = req.goalX;
    int goalY = req.goalY;
    vector<int> start = {startX,startY};
    vector<int> goal = {goalX,goalY};
    //Plan
    vector<vector<int>> path = aSearch(start,goal,graph);
    //Pack response and path viz
    vector<geometry_msgs::Point> finalPath;
    geometry_msgs::Point currentPoint;
    for (auto pathPoint: path){
        int x = pathPoint[xIndex];
        int y = pathPoint[yIndex];
        ROS_INFO("x:%d, y:%d",x,y);
        res.x.push_back(x);
        res.y.push_back(y);
        currentPoint.x = x;
        currentPoint.y = y;
        currentPoint.z = 0;
        finalPath.push_back(currentPoint);
    }
    //Display
    make_path(finalPath);
    //Resets the goals of our graph
    graph->resetGraph();
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "plan_path_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("plan_path", plan);
    ros::Publisher obstacle_pub = n.advertise<visualization_msgs::Marker>("obstacle_markers", 1);
    path_pub = n.advertise<visualization_msgs::Marker>("path_marker", 1);
    ROS_INFO("What is the start and the goal?");
    while(obstacle_pub.getNumSubscribers() == 0){
        ROS_INFO("Waiting for connection to rviz");
    }
    ROS_INFO("Making obstacles");
    make_obstacles(obstacle_pub);
    ros::spin();
    return 0;
    // int xPath;
    // int yPath;
    // underwater_path_planning::Path finalPath;
    // vector<int> start1;
    // start1.push_back(0);
    // start1.push_back(0);
    // vector<int> goal1;
    // goal1.push_back(0);
    // goal1.push_back(8);
    // Graph * graph = new Graph(9,9);

    // graph->printGraph();
    // graph->printObstacles();
    // vector<vector<int>> path1 = aSearch(start1,goal1,graph);
    // for(auto pathPoint : path1) {
    //     xPath = pathPoint[xIndex];
    //     yPath = pathPoint[yIndex];
    //     finalPath.x.push_back(xPath);
    //     finalPath.y.push_back(yPath);
    // }
    // printPath(path1);
    // graph->resetGraph();
    // graph->printGraph();
    // vector<int> start2;
    // start2.push_back(0);
    // start2.push_back(8);
    // vector<int> goal2;
    // goal2.push_back(0);
    // goal2.push_back(0);
    // vector<vector<int>> path2 = aSearch(start2,goal2,graph);
    // printPath(path2);
    // for(auto pathPoint : path2) {
    //     xPath = pathPoint[xIndex];
    //     yPath = pathPoint[yIndex];
    //     finalPath.x.push_back(xPath);
    //     finalPath.y.push_back(yPath);
    // }
    // graph->resetGraph();
    // vector<int> start3;
    // start3.push_back(0);
    // start3.push_back(0);
    // vector<int> goal3;
    // goal3.push_back(5);
    // goal3.push_back(4);
    // vector<vector<int>> path3 = aSearch(start3,goal3,graph);
    // printPath(path3);
    // for(auto pathPoint : path3) {
    //     xPath = pathPoint[xIndex];
    //     yPath = pathPoint[yIndex];
    //     finalPath.x.push_back(xPath);
    //     finalPath.y.push_back(yPath);
    // }
    // graph->resetGraph();
    // graph->resetGraph();
    // vector<int> start4;
    // start4.push_back(5);
    // start4.push_back(4);
    // vector<int> goal4;
    // goal4.push_back(1);
    // goal4.push_back(5);
    // vector<vector<int>> path4 = aSearch(start4,goal4,graph);
    // printPath(path4);
    // for(auto pathPoint : path4) {
    //     xPath = pathPoint[xIndex];
    //     yPath = pathPoint[yIndex];
    //     finalPath.x.push_back(xPath);
    //     finalPath.y.push_back(yPath);
    // }
    // graph->resetGraph();


    // while(ros::ok()) {
    //     planner_pub.publish(finalPath);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
}
