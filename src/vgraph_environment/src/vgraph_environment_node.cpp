#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "ros/ros.h"
// #include "tf"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include<iostream>
#include<fstream>
#include <stack>
#include <math.h>
#include <string>
#include <vector>
# define PI 3.141592

std::pair<int, int> p0;
typedef std::pair<double, std::pair<int,int>> pPair;

struct Node {

    int x, y;

    std::vector<Node> connectionList;

    friend bool operator==(const Node& l, const Node& r)
    {
        return std::tie(l.x, l.y)
             == std::tie(r.x, r.y);
    }
};

float normalize_angle(float angle) {
  float res = angle;
    while(res > PI) {
      res = res - 2.0 * PI;
    }
    while(res < -PI) {
        res = res + 2.0 * PI;
    }
    return res;
}

std::vector<std::vector<std::pair<int, int>>> load_obstacles(std::string object_path)//, std::vector<std::vector<std::pair<int, int>>> &obstacles)
{
    // parsing the obstacles.txt file and getting the information.
    std::fstream fptr;                // File pointer
    fptr.open(object_path);       // Open file with your numbers as input.
    ROS_INFO("after open file");
    std::string tempString = "";      // variable to store the contents read by the pointer
    getline(fptr, tempString);        // read entire line and store the data as a string
    ROS_INFO("tempString1 %s",tempString.c_str());
    int num_obs = std::stoi(tempString, nullptr, 10);//  store the number of obstacles, which is
                                                    //  the first line of the txt file
    std::vector<std::vector<std::pair<int, int>>> obstacles(num_obs);
    for (int i = 0; i < num_obs; i++) {
      getline(fptr, tempString);
      int num_points = std::stoi(tempString, nullptr, 10);   // number of points in an obstacle
      for (int j = 0; j < num_points; j++){
        std::pair<int, int> point; // This variable will store coordinates (x, y)
        getline(fptr, tempString);    // Read in the line from the file.
        ROS_INFO("tempString2 %s",tempString.c_str());
        std::stringstream ss(tempString);    // Convert from a string to a string stream.
        getline(ss, tempString, ' ');       // Get the x coordinate (as a string) with ' ' as the delimiter
        point.first = std::stoi(tempString, nullptr, 10);
        getline(ss, tempString, '\n');      // Get the y coordinate (as a string) with '\n' as the delimiter
        point.second = std::stoi(tempString, nullptr, 10);
        obstacles[i].push_back(point);  // Append the coordinates to the vector.
      }
    }
    fptr.close();
    return obstacles;
}

std::pair<int, int> load_goal(std::string goal_path) { // parsing the goal.txt file and getting the information and should return a vector
    std::fstream gptr;             // File pointer
    gptr.open(goal_path);            // Open file with your numbers as input
    std::string tmpString = " ";      // variable to store the contents read by the pointer
    std::pair<int, int> goal; // This variable will store coordinates (x, y)
    getline(gptr, tmpString);    // Read in the line from the file.
    std::stringstream ss(tmpString);    // Convert from a string to a string stream.
    getline(ss, tmpString, ' ');       // Get the x coordinate (as a string) with ' ' as the delimiter
    goal.first = std::stoi(tmpString, nullptr, 10);
    getline(ss, tmpString, '\n');      // Get the y coordinate (as a string) with '\n' as the delimiter
    goal.second = std::stoi(tmpString, nullptr, 10);
    gptr.close();
    return goal;
}

std::vector<std::vector<std::pair<int, int>>> grow_obstacles (std::vector<std::vector<std::pair<int, int>>> obstacles){

    std::vector<std::vector<std::pair<int, int>>> grown_obstacles(obstacles.size());
    int aabb_sidelen = 36;
    int half = aabb_sidelen / 2;

    for (int o = 0; o < obstacles.size(); o++){
        for (int c = 0; c < obstacles[o].size(); c++){
            int coord1 = obstacles[o][c].first;
            int coord2 = obstacles[o][c].second;
            std::pair<int, int> c1 = std::make_pair(coord1 - half, coord2 + half);
            std::pair<int, int> c2 = std::make_pair(coord1 + half, coord2 + half);
            std::pair<int, int> c3 = std::make_pair(coord1 - half, coord2 - half);
            std::pair<int, int> c4 = std::make_pair(coord1 + half, coord2 - half);
            grown_obstacles[o].push_back(c1);
            grown_obstacles[o].push_back(c2);
            grown_obstacles[o].push_back(c3);
            grown_obstacles[o].push_back(c4);
        }

    }
    return grown_obstacles;
}

// float compute_weight(std::pair<int, int> &e)
// {
//     vector<float> a = e.first;
//     vector<float> b = e.second;
//     return sqrt(pow(a[0]- b[0], 2) + pow(a[1]- b[1], 2))
// }

bool verts_equal(std::pair<double, double> &v1, std::pair<double, double> &v2)
{
    return ((v1.first == v2.first) && (v1.second == v2.second));
}


visualization_msgs::Marker init_marker( int marker_id,  uint32_t marker_type ){
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time();
    m.id = marker_id;
    m.ns = "ns1";
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;

    m.type = marker_type;
    m.scale.x = 0.01;
    m.scale.y = 0.01;
    m.color.g = 1.0;
    m.color.a = 1.0;
    return m;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation (std::pair<double, double> a, std::pair<double, double> b, std::pair<double, double> c) {
  float val = ((b.second- a.second)*(c.first - b.first) - (c.second-b.second)*(b.first - a.first));
  if(val == 0) {
      return 0;
  }
  else if(val > 0) {
      return 1;
  }
  else {
      return 2;
  }
}

// Given three collinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment(std::pair<int, int> p, std::pair<int, int> q, std::pair<int, int> r) {
    if (q.first <= std::max(p.first, r.first) && q.first >= std::min(p.first, r.first) &&
        q.second <= std::max(p.second, r.second) && q.second >= std::min(p.second, r.second))
       return true;

    return false;
}

// The main function that returns true if line segment 'e1 with points a1 and a2'
// and 'e2 with points b1 and b2' intersect.
bool has_intersect(std::pair<geometry_msgs::Point, geometry_msgs::Point> &e1, std::pair<geometry_msgs::Point, geometry_msgs::Point> &e2)
{
    std::pair<double, double> a1 = {e1.first.x, e1.first.y};
    std::pair<double, double> a2 = {e1.second.x, e1.second.y};
    std::pair<double, double> b1 = {e2.first.x, e2.first.y};
    std::pair<double, double> b2 = {e2.second.x, e2.second.y};
    int o1 = orientation(a1, a2, b1);
    int o2 = orientation(a1, a2, b2);
    int o3 = orientation(b1, b2, a1);
    int o4 = orientation(b1, b2, a2);

    if (verts_equal(a1, b1) || verts_equal(a1, b2) || verts_equal(a2, b1) || verts_equal(a2, b2))
        return false;

    if ((o1 != o2) && (o3 != o4))
        return true;

    return false;
}

// A utility function to find next to top in a stack
std::pair<int, int> nextToTop(std::stack<std::pair<int, int>> &S)
{
    std::pair<int, int> p = S.top();
    S.pop();
    std::pair<int, int> res = S.top();
    S.push(p);
    return res;
}

// A utility function to swap two points
void swap(std::pair<int, int> &p1, std::pair<int, int> &p2)
{
    std::pair<int, int> temp = p1;
    p1 = p2;
    p2 = temp;
}

// A utility function to return square of distance
// between p1 and p2
int distSq(std::pair<int, int> p1, std::pair<int, int> p2)
{
    return (p1.first - p2.first)*(p1.first - p2.first) +
          (p1.second - p2.second)*(p1.second - p2.second);
}

// A function used by library function qsort() to sort an array of
// points with respect to the first point
int compare(const void *vp1, const void *vp2)
{
   std::pair<int, int> *p1 = (std::pair<int, int> *)vp1;
   std::pair<int, int> *p2 = (std::pair<int, int> *)vp2;

   // Find orientation
   int o = orientation(p0, *p1, *p2);
   if (o == 0)
     return (distSq(p0, *p2) >= distSq(p0, *p1))? -1 : 1;

   return (o == 2)? -1: 1;
}

// Prints convex hull of a set of n points.
std::vector<geometry_msgs::Point> convexHull(std::vector<std::pair<int, int>> points)
{
  int n = points.size();
   // Find the bottommost point
   int ymin = points[0].second, min_index = 0;
   for (int i = 1; i < n; i++)
   {
     int x = points[i].first;
     int y = points[i].second;
     // Pick the bottom-most or chose the left
     // most point in case of tie
     if ((y < ymin) || (ymin == y &&
         x < points[min_index].first)) {
        ymin = points[i].second;
        min_index = i;
      }
   }

   // Place the bottom-most point at first position
   swap(points[0], points[min_index]);

   // Sort n-1 points with respect to the first point.
   // A point p1 comes before p2 in sorted output if p2
   // has larger polar angle (in counterclockwise
   // direction) than p1
   p0 = points[0];
   qsort(&points[1], n-1, sizeof(std::pair<int, int>), compare);
   // If two or more points make same angle with p0,
   // Remove all but the one that is farthest from p0
   // Remember that, in above sorting, our criteria was
   // to keep the farthest point at the end when more than
   // one points have same angle.
   int m = 1; // Initialize size of modified array
   for (int i=1; i<n; i++) {
       // Keep removing i while angle of i and i+1 is same
       // with respect to p0
       while (i < n-1 && orientation(p0, points[i], points[i+1]) == 0)
          i++;
       points[m] = points[i];
       m++;  // Update size of modified array
   }

   // If modified array of points has less than 3 points,
   // convex hull is not possible
   if (m < 3) {
     std::cerr <<" Not enough points" << std::endl;
     exit(1);
   }

   // Create an empty stack and push first three points
   // to it.
   std::stack<std::pair<int, int>> S;
   S.push(points[0]);
   S.push(points[1]);
   S.push(points[2]);

   // Process remaining m-3 points
   for (int i = 3; i < m; i++) {
     // Keep removing top while the angle formed by
     // points next-to-top, top, and points[i] makes
     // a non-left turn
     while (S.size()>1 && orientation(nextToTop(S), S.top(), points[i]) != 2) {
       S.pop();
     }
     S.push(points[i]);
   }

   // Now stack has the output points, build a vector of points
   std::vector<geometry_msgs::Point> vertices;
   while (!S.empty())
   {
      geometry_msgs::Point point;
      std::pair<int, int> p = S.top();
      point.x = p.first;
      point.y = p.second;
      point.z = 0;
      vertices.push_back(point);
      S.pop();
   }
   return vertices;
}

// A* Planning
// Go through each list of hull_verts as points in grid map
// publish map with markers
// std::list<pPair> Wildfire::AStarNodeMap( Node src, Node dest, std::vector<Node> nodeList ){
    // ROS_INFO("A* Node Map SRC x:%f y:%f Dest x:%f y:%f",src.x,src.y,dest.x,dest.y);
    // // map_.add(map);
    // publish();
    // // Path list with pair of heading (rads - double), and position (x, y)
    // std::list<pPair> pathList;

    // // If the source is out of range
    // if (isValid(src.x, src.y) == false) {
    //     ROS_INFO("Source is invalid\n");
    //     return pathList;
    // }
 
    // // If the destination is out of range
    // if (isValid(dest.x, dest.y) == false) {
    //     ROS_INFO("Destination is invalid\n");
    //     return pathList;
    // }
 
    // // Either the source or the destination is blocked
    // if (isUnBlocked( src.x, src.y) == false
    //     || isUnBlocked( dest.x, dest.y)
    //            == false) {
    //     ROS_INFO("Source or the destination is blocked\n");
    //     return pathList;
    // }
 
    // // If the destination cell is the same as source cell
    // if (isDestination(src.x, src.y, dest)
    //     == true) {
    //     ROS_INFO("We are already at the destination\n");
    //     return pathList;
    // }
    

    // Create a closed list and initialise it to false which
    // means that no cell has been included yet This closed
    // list is implemented as a boolean 2D array
//     bool closedList[gridLength][gridLength];
//     memset(closedList, false, sizeof(closedList));
 
//     // Declare a 2D array of structure to hold the details
//     // of that cell
//     cell cellDetails[gridLength][gridLength];
 
//     int i, j;
//     // if (enableLogging){
//     //     ROS_INFO("Before cell details setup");
//     // }
//     for (i = 0; i < gridLength; i++) {
//         for (j = 0; j < gridLength; j++) {
//             cellDetails[i][j].f = FLT_MAX;
//             cellDetails[i][j].g = FLT_MAX;
//             cellDetails[i][j].h = FLT_MAX;
//             cellDetails[i][j].parent_i = -1;
//             cellDetails[i][j].parent_j = -1;
//             cellDetails[i][j].theta = 0;
//         }
//     }
//     // if (enableLogging){
//     //     ROS_INFO("After cell details setup");
//     // }

//     // Initialising the parameters of the starting node
//     i = src.x, j = src.y;
//     cellDetails[i][j].f = 0.0;
//     cellDetails[i][j].g = 0.0;
//     cellDetails[i][j].h = 0.0;
//     cellDetails[i][j].parent_i = i;
//     cellDetails[i][j].parent_j = j;
//     cellDetails[i][j].theta = 0;

//     int di, dj;
//     di = dest.x, dj = dest.y;
//     // cellDetails[di][dj].theta = 0;
//     /*
//      Create an open list having information as-
//      <f, <i, j>>
//      where f = g + h,
//      and i, j are the row and column index of that cell
//      Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
//      This open list is implemented as a set of pair of
//      pair.*/
//     std::list<pPair> openList;
 
//     // Put the starting cell on the open list and set its
//     // 'f' as 0
//     pPair firstPair = std::make_pair(0.0, Position(i, j));
//     openList.push_back(firstPair);
 
//     // We set this boolean value as false as initially
//     // the destination is not reached.
//     bool foundDest = false;
        
//     int xNew, yNew;
//     double thetaNew;
//     double vMin{-2};
//     double vMax{10};

//     double wMin{-7};
//     double wMax{7};
        
//     double L{3};

//     while (!openList.empty()) {
//         pPair p = *openList.begin();
 
//         // Remove this vertex from the open list
//         openList.erase(openList.begin());
 
//         // Add this vertex to the closed list
//         i = p.second.x();
//         j = p.second.y();
//         closedList[i][j] = true;
  
//         // To store the 'g', 'h' and 'f' of the 8 successors
//         double gNew, hNew, fNew;

//         // set min and max Ul and Ur

//         // Nested for loops for both (min-max Ul, min-max Ur) (custom step)
//         for (Node n: nodeMap_){

//             xNew = n.x;
//             yNew = n.y;
//             if (isValid(xNew, yNew) == true) {
//                 // If the destination cell is the same as the
//                 // current successor
//                 ROS_INFO_COND(enableLogging,"New Position x:%d y:%d", xNew, yNew);
//                 if ((isDestination(xNew, yNew, dest) == true))
//                 {
//                     // Set the Parent of the destination cell
//                     cellDetails[xNew][yNew].parent_i = i;
//                     cellDetails[xNew][yNew].parent_j = j;
//                     ROS_INFO("The destination cell is found\n");
//                     tracePath(cellDetails, dest, map);
//                     foundDest = true;
//                     pathList = getPath(cellDetails, dest);
//                     return pathList;
//                 }
//                 // If the successor is already on the closed
//                 // list or if it is blocked, then ignore it.
//                 // Else do the following
//                 else if (closedList[xNew][yNew] == false
//                         && isUnBlocked( xNew, yNew)
//                                 == true) {
//                     gNew = calculateGValue(xNew, yNew, cellDetails[i][j].g, src);
//                     hNew = calculateHValue(xNew, yNew, dest);
//                     fNew = gNew + hNew;
//                     if (enableLogging){
//                         ROS_INFO("fNew %f", fNew);
//                     }
//                     // If it isn’t on the open list, add it to
//                     // the open list. Make the current square
//                     // the parent of this square. Record the
//                     // f, g, and h costs of the square cell
//                     //                OR
//                     // If it is on the open list already, check
//                     // to see if this path to that square is
//                     // better, using 'f' cost as the measure.
//                     if (cellDetails[xNew][yNew].f == FLT_MAX
//                         || cellDetails[xNew][yNew].f > fNew) {
//                         openList.push_back(std::make_pair(
//                             fNew, Position(xNew, yNew)));
    
//                         // Update the details of this cell
//                         cellDetails[xNew][yNew].f = fNew;
//                         cellDetails[xNew][yNew].g = gNew;
//                         cellDetails[xNew][yNew].h = hNew;
//                         cellDetails[xNew][yNew].parent_i = i;
//                         cellDetails[xNew][yNew].parent_j = j;
//                         // cellDetails[xNew][yNew].theta = thetaNew;
//                         if (enableLogging){
//                             ROS_INFO("Add to openList x: %d y: %d", xNew, yNew);
//                         }
//                     }
//                 }
//             }
//         }        
//     }
//     // When the destination cell is not found and the open
//     // list is empty, then we conclude that we failed to
//     // reach the destination cell. This may happen when the
//     // there is no way to destination cell (due to
//     // blockages)
//     if (foundDest == false)
//         ROS_INFO("Failed to find the Destination Cell\n");
//         ROS_INFO("Last Position x: %d y: %d", xNew, yNew);

//     pathList = getPath(cellDetails, dest);
//     return pathList;
// }

class Vgraph {
  public :
    Vgraph(int argc, char** argv) {
      ros::init(argc, argv, "vgraph_environment");
      ros::NodeHandle n;
      ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("vgraph_markerarr", 10);
      // ros::Publisher cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
      ros::Rate loop_rate(30);

      // tf::TransformListener this->tf_listener;
      // loop_rate.sleep(2);
      //
      // this->odom_frame = "/odom";
      // tf::StampedTransform transform;
      // try
      // {
      //     this->tf_listener.lookupTransform(this->odom_frame, "/base_footprint",ros::Time(0), transform);
      //     this->base_frame = '/base_footprint';
      // }
      // catch(tf::TransformException ex)
      // {
      //     try
      //     {
      //         this->tf_listener.lookupTransform(this->odom_frame, "/base_link",ros::Time(0), transform);
      //         this->base_frame = '/base_link';
      //     }
      //     catch(tf::TransformException ex)
      //     {
      //         ROS_ERROR("%s",ex.what());
      //         ros::Duration(1.0).sleep();
      //     }
      // }
      ROS_INFO("Start");
      float scale_factor = 100;
      // std::string object_path = "/home/prajwal/Desktop/rbe500-ros/src/vgraph_environment/src/obstacles.txt";
      std::string object_path = "/home/chris/git/Multi_Layer_Motion_Planning/src/vgraph_environment/src/obstacles.txt";
      ROS_INFO("parse object1");
      std::vector<std::vector<std::pair<int, int>>> obstacles = load_obstacles(object_path);
      ROS_INFO("parse object2");

      std::vector<std::vector<std::pair<int, int>>> grown_obstacles = grow_obstacles(obstacles);

      // std::string goal_path = "/home/prajwal/Desktop/rbe500-ros/src/vgraph_environment/src/goal.txt";
      std::string goal_path = "/home/chris/git/Multi_Layer_Motion_Planning/src/vgraph_environment/src/goal.txt";
      ROS_INFO("parse goal");
      std::pair<int, int> goal = load_goal(goal_path);
      std::pair<int, int> start {-200,0};
      ROS_INFO("marker array");
      visualization_msgs::MarkerArray marker_arr;
      int marker_id = 0;

      std::vector<geometry_msgs::Point> vertices;
      std::vector<std::vector<geometry_msgs::Point>> hull_verts;
      std::vector<std::pair<geometry_msgs::Point, geometry_msgs::Point>> hull_edges;
      std::vector<geometry_msgs::Point> points;
      visualization_msgs::Marker m = init_marker(marker_id, visualization_msgs::Marker::LINE_LIST);
      marker_id ++;

      std::vector<Node> nodeList;

      // Draw convex hull around obstacles
      // some initialization as per .py file
      for (int i=0; i < grown_obstacles.size(); i++) {
        std::vector<geometry_msgs::Point> verts = convexHull(grown_obstacles[i]);
        int size = verts.size();
        vertices.clear();
        for (int j = 0; j < size; j++) {
          geometry_msgs::Point p1;
          geometry_msgs::Point p2;
          p1.x = verts[j].x / scale_factor;
          p1.y = verts[j].y / scale_factor;
          p1.z = verts[j].z / scale_factor;
          if (j == verts.size() - 1) {
            p2.x = verts[0].x / scale_factor;
            p2.y = verts[0].y / scale_factor;
            p2.z = verts[0].z / scale_factor;

          }
          else {
            p2.x = verts[j+1].x / scale_factor;
            p2.y = verts[j+1].y / scale_factor;
            p2.z = verts[j+1].z / scale_factor;
          }
          vertices.push_back(p1);
          Node n;
          n.x = p.x;
          n.y = p.y;
          nodeList.push_back(n);
          points.push_back(p1);
          points.push_back(p2);
          std::pair<geometry_msgs::Point, geometry_msgs::Point> edge = {p1, p2};
          hull_edges.push_back(edge);
        }
        hull_verts.push_back(vertices);
      }

      m.points = points;            //Push all points of convex hull into marker array
      marker_arr.markers.push_back(m);


      // // Draw paths
      // // some initialization as per .py file

      visualization_msgs::Marker marker = init_marker(marker_id, visualization_msgs::Marker::LINE_LIST);
      marker_id ++;
      points.clear();
      vertices.clear();

      geometry_msgs::Point start_point, goal_point;
      start_point.x = start.first / scale_factor;
      start_point.y = start.second / scale_factor;
      start_point.z = 0;

      goal_point.x = goal.first / scale_factor;
      goal_point.y = goal.second / scale_factor;
      goal_point.z = 0;
      std::vector<geometry_msgs::Point> goal_vector {goal_point};
      std::vector<geometry_msgs::Point> start_vector {start_point};
      hull_verts.push_back(start_vector);
      hull_verts.push_back(goal_vector);

      std::vector<geometry_msgs::Point> temp;
      for(int i = 0; i < hull_verts.size() - 1; i++) {
        for (int j = i + 1; j < hull_verts.size(); j++) {
          for (int k = 0; k < hull_verts[i].size(); k++) {
            for (int l = 0; l < hull_verts[j].size(); l++) {
              std::pair<geometry_msgs::Point, geometry_msgs::Point> edge = {hull_verts[i][k], hull_verts[j][l]};
              bool flag = true;
              for (int p = 0; p < hull_edges.size(); p++) {
                if (has_intersect(hull_edges[p], edge)) {
                  flag = false;
                  break;
                }
              }

              if (flag == true) {
                points.push_back(hull_verts[i][k]);
                points.push_back(hull_verts[j][l]);
              }
            }
          }
        }
      }
      marker.points = points;
      marker_arr.markers.push_back(marker);

      while (ros::ok) {
        marker_pub.publish(marker_arr);
      }


  //     float this->linear_speed = 0.15;
  //     float this->angular_speed = 0.5;
  //     float this->angular_tolerance = 0.1;
  //
  //
  // }
  //
  // void translate(goal_distance)
  // {
  //     geometry_msgs::Twist move_cmd;
  //     move_cmd.linear.x = this->linear_speed;
  //     // need to call get_odom function. Currently assuming we got position and rotation value.
  //     float x_start = position.x;
  //     float y_start = position.y;
  //     float distance = 0;
  //     while(distance < goal_distance)
  //     {
  //         this->cmd_vel.publish(move_cmd);
  //         // need to call get_odom function. Currently assuming we got position and rotation value.
  //         distance = sqrt(pow(position.x- x_start, 2) + pow(position.y- y_start, 2))
  //     }
  //     move_cmd.linear.x = 0;
  //     this->cmd_vel.publish(move_cmd);
  //     loop_rate.sleep();
  //
  // }
  //
  // void rotate(goal_angle)
  // {
  //     geometry_msgs::Twist move_cmd;
  //     move_cmd.angular.z = this->angular_speed;
  //     if(goal_angle > 0)
  //     {
  //         this->angular_speed = -(this->angular_speed);
  //     }
  //
  //     // need to call get_odom function. Currently assuming we got position and rotation value.
  //     float last_angle = rotation;
  //     float turn_angle = 0;
  //     while(abs(turn_angle + this->angular_tolerance) < abs(goal_angle))
  //     {
  //         this->cmd_vel.publish(move_cmd);
  //         // need to call get_odom function. Currently assuming we got position and rotation value.
  //         float delta_angle = normalize(rotation - last_angle);
  //         turn_angle = turn_angle + delta_angle;
  //         last_angle = rotation;
  //
  //     }
  //     move_cmd.angular.z = 0;
  //     this->cmd_vel.publish(move_cmd);
  //     loop_rate.sleep();
  //
  //
  // }
  //
  // void get_odom()
  // {
  //     geometry_msgs::Point
  //     try
  //     {
  //         this->tf_listener.lookupTransform(this->odom_frame, this->base_frame,ros::Time(0), transform); // Not  able to figure what will be the output of this line. Need to figure something for that.
  //     }
  // }
  //
  // void shutdown()
  // {
  //     ROS_INFO("Stopping the robot !!");
  //     geometry_msgs::Twist move_cmd;
  //     this->cmd_vel.publish(move_cmd);
  //     loop_rate.sleep();
  // }
  //

    }
};




int main(int argc, char** argv)
{
    Vgraph vgraph(argc, argv);
}
