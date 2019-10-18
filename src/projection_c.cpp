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


  if (exp_num >= 9 && exp_num <= 12) {
    ///// decide image size in real world
    float size = 800 / 2;
    int ran = rand() % 10;
    if (ran % 2 == 0) {
      ran = ran + 1;
    } else {

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
      cv::Mat M = (cv::Mat_<double>(3,3) << -0.46373082766843, -0.391328023159111, 901.1663818359388, 0.05576542651957173, -0.07783323985820123, 386.7452392578139, -0.0001504435708675367, 0.0002103010190235412, 1);
      cv::warpPerspective( source_img, warp_img, M, source_img.size());
      cv::namedWindow( "screen_c" , CV_WINDOW_NORMAL);
      cv::setWindowProperty("screen_c",CV_WND_PROP_FULLSCREEN,CV_WINDOW_FULLSCREEN);
      cv::imshow("screen_c", warp_img);
      cv::waitKey(1);

      rate.sleep();
      }
      cv::destroyWindow("screen_c");


    }


}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "exp_9101112");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("finish_pantilt", 1000, &Callback);

  ros::Rate rate(20);

  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
    }


  return 0;
}
