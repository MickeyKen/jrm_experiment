#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include <thread>

int data_base = 0;
void Callback(const std_msgs::Int16& msg)
{
  //std::cout << msg.data << std::endl;
  data_base=msg.data;
  ros::NodeHandle n;

  int exp_num = 0;
  int fin_switch = 1;

  n.getParam("/exp_num", exp_num);
  n.setParam("exp_miki_img/switch", 1);

  ros::Rate rate(20);


  if (exp_num >= 1 && exp_num <= 4) {
    ///// decide image size in real world
    float size = 800 / 2;
    int ran = rand() % 10;
    if (ran % 2 == 0) {

    } else {
      ran = ran + 1;

    }
    ///// get image and resize projectr size
    std::string file_dir = "/home/ud/catkin_ws/src/jrm_experiment/src/image/";
    std::string input_file_path = file_dir + std::to_string(ran) + ".png";
    cv::Mat source_img = cv::imread(input_file_path, cv::IMREAD_UNCHANGED);
    int ColumnOfNewImage = 1024;
    int RowsOfNewImage = 768;
    ///// main function
    while (ros::ok()) {

      ///// switch
      n.getParam("exp_miki_img/switch", fin_switch);
      if (fin_switch == 0) {
        break;
      }
      cv::Mat warp_img(cv::Size(1024, 768), CV_8U, cv::Scalar::all(0));
      resize(source_img, source_img, cv::Size(ColumnOfNewImage,RowsOfNewImage));
      cv::Mat M = (cv::Mat_<double>(3,3) << 0.1328612051691586, 0.8424816092172287, 186.7617340087891, -0.1048375316667271, 0.001463219311317654, 486.4222412109378, 0.0002541468389892483, -3.799551891974472e-06, 1);
      cv::warpPerspective( source_img, warp_img, M, source_img.size());
      cv::namedWindow( "screen_24" ,CV_WINDOW_NORMAL);
      cv::setWindowProperty("screen_24",CV_WND_PROP_FULLSCREEN,CV_WINDOW_FULLSCREEN);
      cv::imshow("screen_24", warp_img);
      cv::waitKey(1);

      rate.sleep();
      }
      cv::destroyWindow("screen_24");


    }


}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "exp_1234");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("finish_pantilt", 1000, &Callback);

  ros::Rate rate(20);

  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
    }


  return 0;
}
