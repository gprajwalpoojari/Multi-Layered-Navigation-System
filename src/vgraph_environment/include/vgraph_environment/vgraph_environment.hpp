#include <iostream>
#include <ros/ros.h>
#include <stack>
#include <vector>
#include <string>

typedef std::vector<std::pair<double,double>> path;

struct Node {

    double x, y;

    int index;

    std::vector<Node> connectionList;

    friend bool operator==(const Node& l, const Node& r)
    {
        return std::tie(l.x, l.y)
             == std::tie(r.x, r.y);
    }
};

struct cell {
    
    Node parent_node, node;

    double f, g, h;

};

bool enableLogging = false;

float normalize_angle(float angle);

std::vector<std::vector<std::pair<int, int>>> load_obstacles(std::string object_path);

// parsing the goal.txt file and getting the information and should return a vector
std::pair<int, int> load_goal(std::string goal_path);

std::vector<std::vector<std::pair<int, int>>> grow_obstacles (std::vector<std::vector<std::pair<int, int>>> obstacles);

bool verts_equal(std::pair<double, double> &v1, std::pair<double, double> &v2);

visualization_msgs::Marker init_marker( int marker_id,  uint32_t marker_type );

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation (std::pair<double, double> a, std::pair<double, double> b, std::pair<double, double> c);

// Given three collinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment(std::pair<int, int> p, std::pair<int, int> q, std::pair<int, int> r);

// The main function that returns true if line segment 'e1 with points a1 and a2'
// and 'e2 with points b1 and b2' intersect.
bool has_intersect(std::pair<geometry_msgs::Point, geometry_msgs::Point> &e1, std::pair<geometry_msgs::Point, geometry_msgs::Point> &e2);

// A utility function to find next to top in a stack
std::pair<int, int> nextToTop(std::stack<std::pair<int, int>> &S);

// A utility function to swap two points
void swap(std::pair<int, int> &p1, std::pair<int, int> &p2);

// A utility function to return square of distance
// between p1 and p2
int distSq(std::pair<int, int> p1, std::pair<int, int> p2);

// A function used by library function qsort() to sort an array of
// points with respect to the first point
int compare(const void *vp1, const void *vp2);

double calculateHValue(double row, double col, double destX, double destY );

double calculateGValue(double row, double col, double g, double srcX, double srcY );

/*!
 * @brief Get the Node at the given position
 * 
 * @param x x-position
 * @param y y-position
 * @param nodeList list of nodes to search
 * @return Node 
 */
Node getNodeFromPos ( double x, double y, std::vector<Node> nodeList );

path getPath(cell cellDetails[], Node dest, std::vector<Node> nodeList);

// Covert path list to pairs for marker line list
std::vector<geometry_msgs::Point> getMarkerPoints ( path pathList, double scaleFactor );

std::vector<Node> replaceNodeInList(Node n, std::vector<Node> nodeList );

/*!
 * @brief Print out each of the nodes in the list including their x,y position 
 *        and size of connection list
 * 
 * @param nodeList 
 */
void outputAllNodes ( std::vector<Node> nodeList );

// Prints convex hull of a set of n points.
std::vector<geometry_msgs::Point> convexHull(std::vector<std::pair<int, int>> points);

// A* Planning
// Go through each list of hull_verts as points in grid map
// publish map with markers
path AStarNodeMap( Node src, Node dest, std::vector<Node> nodeList );

class Vgraph
{
public:
    Vgraph(int argc, char** argv);
private:
};