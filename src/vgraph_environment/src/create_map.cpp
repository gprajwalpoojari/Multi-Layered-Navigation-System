#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string.h>
#include "opencv2/opencv.hpp"

std::vector<std::vector<std::pair<int, int>>> load_obstacles(std::string object_path)//, std::vector<std::vector<std::pair<int, int>>> &obstacles)
{
  // parsing the world_obstacles.txt file and getting the information.
  // As obstacles is passed by reference, we dont have to return it.
  std::fstream fptr;                // File pointer
  fptr.open(object_path);       // Open file with your numbers as input.
  std::string tempString = "";      // variable to store the contents read by the pointer
  getline(fptr, tempString);        // read entire line and store the data as a string
  int num_obs = stoi(tempString, nullptr, 10);  //  store the number of obstacles, which is                                                  //  the first line of the txt file

  std::vector<std::vector<std::pair<int, int>>> obstacles(num_obs);
  for (int i = 0; i < num_obs; i++) {
    getline(fptr, tempString);
    int num_points = stoi(tempString, nullptr, 10);   // number of points in an obstacle
    for (int j = 0; j < num_points; j++){
      std::pair<int, int> point; // This variable will store coordinates (x, y)
      getline(fptr, tempString);    // Read in the line from the file.
      std::stringstream ss(tempString);    // Convert from a string to a string stream.
      getline(ss, tempString, ' ');       // Get the x coordinate (as a string) with ' ' as the delimiter
      point.first = stoi(tempString, nullptr, 10);
      getline(ss, tempString, '\n');      // Get the y coordinate (as a string) with '\n' as the delimiter
      point.second = stoi(tempString, nullptr, 10);
      obstacles[i].push_back(point);  // Append the coordinates to the vector.
    }
  }
  fptr.close();
  return obstacles;
}

std::pair<int, int> load_goal(std::string goal_path) { // parsing the goal.txt file and getting the information and should return a vector
  std::fstream gptr;             // File pointer
  gptr.open(goal_path);            // Open file with your numbers as input
  std::string tmpString = "";      // variable to store the contents read by the pointer
  std::pair<int, int> goal; // This variable will store coordinates (x, y)
  getline(gptr, tmpString);    // Read in the line from the file.
  std::stringstream ss(tmpString);    // Convert from a string to a string stream.
  getline(ss, tmpString, ' ');       // Get the x coordinate (as a string) with ' ' as the delimiter
  goal.first = stoi(tmpString, nullptr, 10);
  getline(ss, tmpString, '\n');      // Get the y coordinate (as a string) with '\n' as the delimiter
  goal.second = stoi(tmpString, nullptr, 10);
  gptr.close();
  return goal;
}

int main() {

  //******************************LOAD OBSTACLES FROM TXT FILE******************************//
  std::string object_path = "obstacles.txt";
  std::vector<std::vector<std::pair<int, int>>> obstacles = load_obstacles(object_path);

  //******************************//LOAD OBSTACLES FROM TXT FILE//******************************//


  //******************************LOAD GOAL FROM TXT FILE******************************//
  std::string goal_path = "goal.txt";
  std::pair<int, int> goal = load_goal(goal_path);
  //******************************//LOAD GOAL FROM TXT FILE//******************************//


  //**************************************LOAD START***************************************//
  std::pair<int, int> start = {-200, 0};
  //**************************************//LOAD START//***************************************//

  //**************************************TRANSFORM OBSTACLE COORDINATES***************************************//
  std::vector<std::vector<std::pair<int, int>>> transformed_obstacles (obstacles.size());
  for (int i = 0; i < obstacles.size(); i++) {
    for (int j = 0; j < obstacles[i].size(); j++) {
      transformed_obstacles[i].push_back(obstacles[i][j]);
      transformed_obstacles[i][j].first = 300 + transformed_obstacles[i][j].first;
      transformed_obstacles[i][j].second = 300 - transformed_obstacles[i][j].second;
    }
  }
  //**********************************//TRANSFORM OBSTACLE COORDINATES//***************************************//


  //**************************************LOAD BLANK IMAGE***************************************//

  // Height  = 500 pixels, Width = 1000 pixels
  // (0, 0, 0) assigned for Blue, Green and Red plane respectively.
  cv::Mat img(600, 1200, CV_8UC3, cv::Scalar(0, 0, 0));     // CV_8UC3 depicts : (3 channels,8 bit image depth
  std::cout << img.rows << " " << img.cols << std::endl;
  // for (int y = 0; y < img.rows; y++) {
  //   for (int x = 0; x < img.cols; x++) {
      // cv::Vec3b &color = img.at<cv::Vec3b>(y,x);
      // // ... do something to the color ....
      // color[0] = 255;
      // color[1] = 255;
      // color[2] = 0;
    // }
  // }
  // check whether the image is loaded or not
  if (img.empty()) {
    std::cout << "\n Image not created. You have done something wrong. \n";
    return -1;    // Unsuccessful.
  }
  //
  // cv::namedWindow("A_good_name", CV_WINDOW_AUTOSIZE);
  //  //
  //  // // first argument: name of the window
  //  // // second argument: image to be shown(Mat object)
  // cv::imshow("A_good_name", img);
  //  //
  // cv::waitKey(0); //wait infinite time for a keypress
  //  //
  //  // // destroy the window with the name, "MyWindow"
  // cv::destroyWindow("A_good_name");

   //return 0;


   //********************************BRESENHAM LINE DRAWING ALGORITHM*********************************//
    std::vector<std::vector<std::vector<std::pair<int, int>>>> rasterized_line (transformed_obstacles.size());
   for (int i = 0; i < transformed_obstacles.size(); i++) {
     for (int j = 0; j < transformed_obstacles[i].size(); j++) {
       std::pair<int, int> points[2];
       points[0] = {transformed_obstacles[i][j].first, transformed_obstacles[i][j].second};
       if (j == transformed_obstacles[i].size() - 1) {
         points[1] = {transformed_obstacles[i][0].first, transformed_obstacles[i][0].second};
       }
       else {
         points[1] = {transformed_obstacles[i][j+1].first, transformed_obstacles[i][j+1].second};
       }
       std::vector<std::pair<int, int>> line;
       line.push_back(points[0]);
       int x = points[0].first;
       int y = points[0].second;
       int x2 = points[1].first;
       int y2 = points[1].second;
       int dx = abs(x2 - x);
       int dy = -abs(y2 - y);
       int sx = x < x2 ? 1 : -1;
       int sy = y < y2 ? 1 : -1;
       int err = dx + dy;
       while (x != x2 || y != y2) {
         int e2 = 2*err;
         if (e2 >= dy) {
           err += dy;
           x += sx;
         }
         if (e2 <= dx) {
           err += dx;
           y+=sy;
         }
         cv::Vec3b &color = img.at<cv::Vec3b>(y,x);
          // ... do something to the color ....
          color[0] = 255;
          color[1] = 255;
          color[2] = 0;
          line.push_back({x, y});
       }
       line.push_back(points[1]);
       rasterized_line[i].push_back(line);
     }
   }
   //********************************//BRESENHAM LINE DRAWING ALGORITHM//*********************************//


   //********************************BRESENHAM CIRCLE DRAWING ALGORITHM*********************************//
   std::pair<int, int> transformed_start = {300 + start.first, 300 - start.second};
   std::pair<int, int> transformed_goal = {300 + goal.first, 300 - goal.second};
   std::vector<std::vector<std::pair<int, int>>> rasterized_circle (2);
   int radius = 5;
   int x = 0;
   int y = radius;
   int decision = 3 - 2* radius;
   rasterized_circle[0].push_back({x + transformed_start.first, y + transformed_start.second});
   rasterized_circle[0].push_back({x + transformed_start.first, -y + transformed_start.second});
   rasterized_circle[0].push_back({y + transformed_start.first, x + transformed_start.second});
   rasterized_circle[0].push_back({-y + transformed_start.first, x + transformed_start.second});
   rasterized_circle[1].push_back({x + transformed_goal.first, y + transformed_goal.second});
   rasterized_circle[1].push_back({x + transformed_goal.first, -y + transformed_goal.second});
   rasterized_circle[1].push_back({y + transformed_goal.first, x + transformed_goal.second});
   rasterized_circle[1].push_back({-y + transformed_goal.first, x + transformed_goal.second});
   while (x < y) {
     if (decision < 0) {
       decision = decision + 4 * x + 6;
       x = x + 1;
     }
     else {
       decision = decision + 4 * (x - y) + 10;
       x = x + 1;
       y = y - 1;
     }
     rasterized_circle[0].push_back({x + transformed_start.first, y + transformed_start.second});
     rasterized_circle[0].push_back({y + transformed_start.first, x + transformed_start.second});
     rasterized_circle[0].push_back({x + transformed_start.first, -y + transformed_start.second});
     rasterized_circle[0].push_back({y + transformed_start.first, -x + transformed_start.second});
     rasterized_circle[0].push_back({-x + transformed_start.first, y + transformed_start.second});
     rasterized_circle[0].push_back({-y + transformed_start.first, x + transformed_start.second});
     rasterized_circle[0].push_back({-x + transformed_start.first, -y + transformed_start.second});
     rasterized_circle[0].push_back({-y + transformed_start.first, -x + transformed_start.second});
     rasterized_circle[1].push_back({x + transformed_goal.first, y + transformed_goal.second});
     rasterized_circle[1].push_back({y + transformed_goal.first, x + transformed_goal.second});
     rasterized_circle[1].push_back({x + transformed_goal.first, -y + transformed_goal.second});
     rasterized_circle[1].push_back({y + transformed_goal.first, -x + transformed_goal.second});
     rasterized_circle[1].push_back({-x + transformed_goal.first, y + transformed_goal.second});
     rasterized_circle[1].push_back({-y + transformed_goal.first, x + transformed_goal.second});
     rasterized_circle[1].push_back({-x + transformed_goal.first, -y + transformed_goal.second});
     rasterized_circle[1].push_back({-y + transformed_goal.first, -x + transformed_goal.second});
   }
   for (int i = 0; i < rasterized_circle.size(); i++) {
     for (int j = 0; j < rasterized_circle[i].size(); j++) {
       cv::Vec3b &color = img.at<cv::Vec3b>(rasterized_circle[i][j].second,rasterized_circle[i][j].first);
       // // ... do something to the color ....
       color[0] = 0;
       color[1] = 0;
       color[2] = 255;
     }
   }
   //********************************//BRESENHAM CIRCLE DRAWING ALGORITHM//*********************************//


   //********************************FILL CIRCLE ALGORITHM*********************************//
   for (int i = 0; i < rasterized_circle.size(); i++) {
     int x0[img.cols], x1[img.cols];
     for (int i = 0; i < img.cols; i++) {
       x0[i] = -1;
       x1[i] = -1;
     }
     for (int j = 0; j < rasterized_circle[i].size(); j++) {
       int x = rasterized_circle[i][j].first;
       int y = rasterized_circle[i][j].second;
       if (x0[y] == -1) {
         x0[y] = x;
         x1[y] = x;
       }
       else {
         x0[y] = std::min(x0[y], x);
         x1[y] = std::max(x1[y], x);
       }
     }
     for (int y = 0; y < img.cols; y++) {
       for (int x = x0[y]; x < x1[y] && x != -1; x++) {
          cv::Vec3b &color = img.at<cv::Vec3b>(y,x);
          // ... do something to the color ....
          color[0] = 0;
          color[1] = 0;
          color[2] = 255;
       }
     }
   }
   //********************************//FILL CIRCLE ALGORITHM//*********************************//


   //********************************FILL POLYGON ALGORITHM*********************************//


   for (int i = 0; i < rasterized_line.size(); i++) {
     int x0[img.cols], x1[img.cols];
     for (int i = 0; i < img.cols; i++) {
       x0[i] = -1;
       x1[i] = -1;
     }
     for (int j = 0; j < rasterized_line[i].size(); j++) {
       for (int k = 0; k < rasterized_line[i][j].size(); k++) {
         int x = rasterized_line[i][j][k].first;
         int y = rasterized_line[i][j][k].second;
         if (x0[y] == -1) {
           x0[y] = x;
           x1[y] = x;
         }
         else {
           x0[y] = std::min(x0[y], x);
           x1[y] = std::max(x1[y], x);
         }
       }
     }
     for (int y = 0; y < img.cols; y++) {
       for (int x = x0[y]; x < x1[y] && x != -1; x++) {
          cv::Vec3b &color = img.at<cv::Vec3b>(y,x);
          // ... do something to the color ....
          color[0] = 255;
          color[1] = 255;
          color[2] = 0;
       }
     }
   }

   //********************************//FILL POLYGON ALGORITHM//*********************************//
   cv::namedWindow("Obstacle_Map", CV_WINDOW_AUTOSIZE);
    //
    // // first argument: name of the window
    // // second argument: image to be shown(Mat object)
   cv::imshow("Obstacle_Map", img);
  //  cv::imwrite("/home/prajwal/Desktop/rbe500-ros/src/vgraph_environment/map/Obstacle_Map.png", img);
   cv::imwrite("/home/chris/git/Multi_Layer_Motion_Planning/src/vgraph_environment/map/Obstacle_Map.png", img);
    //
   cv::waitKey(0); //wait infinite time for a keypress
    //
    // // destroy the window with the name, "MyWindow"
   cv::destroyWindow("A_good_name");

   //**************************************//LOAD BLANK IMAGE//***************************************//
  return 0;
}
