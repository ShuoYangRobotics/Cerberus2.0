#include "fusion/VILOFusion.hpp"

VILOFusion::VILOFusion(ros::NodeHandle nh) { 
  nh_ = nh;
  Utils::readParameters(nh);
}

void VILOFusion::loop() {
  /*
   * The loop function do several things: 
        sync stereo image and input them to VILOEstimator
        sync proprioceptive sensors
        run MIPOEstimator
        input necessary information to VILOEstimator
        record MIPO and VILO results
   */
  const double LOOP_DT = 2.5; // 400Hz
  prev_loop_time = ros::Time::now().toSec();
  prev_esti_time = ros::Time::now().toSec();

  while (1) {
    /* record start time */
    auto loop_start = std::chrono::system_clock::now();
    auto ros_loop_start = ros::Time::now().toSec();

    /* get a bunch of ros objects */
    double curr_loop_time = ros::Time::now().toSec();
    double dt_ros = curr_loop_time - prev_loop_time;
    prev_loop_time = curr_loop_time;
    std::cout << "dt_ros: " << dt_ros << std::endl;
    if (dt_ros == 0) {
      continue;
    }

    /* estimator logic */
    inputImages();

    /* record end time */
    auto loop_end = std::chrono::system_clock::now();
    auto loop_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
        loop_end - loop_start);
    std::cout << "total loop time: " << loop_elapsed.count() << std::endl;

    auto ros_loop_end = ros::Time::now().toSec();
    auto ros_loop_elapsed = ros_loop_end - ros_loop_start;

    /* sleep a while to make sure the loop run at LOOP_DT */
    if (ros_loop_elapsed * 1000 >= LOOP_DT) {
      // do not sleep
      std::cout << "loop computation time is longer than desired dt, optimize "
                   "code or increase LOOP_DT"
                << std::endl;
    } else {
      double sleep_time = LOOP_DT * 0.001 - ros_loop_elapsed;
      ros::Duration(sleep_time).sleep();
    }
  }
  return;
}




// private functions
void VILOFusion::img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    const std::lock_guard<std::mutex> lock(mtx_image);
    img0_buf.push(img_msg);
}

void VILOFusion::img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    const std::lock_guard<std::mutex> lock(mtx_image);
    img1_buf.push(img_msg);
}
cv::Mat VILOFusion::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

void VILOFusion::inputImages() {
  const std::lock_guard<std::mutex> lock(mtx_image);
  cv::Mat image0, image1;
  std_msgs::Header header;
  double time = 0;
  if (!img0_buf.empty() && !img1_buf.empty())
  {
      double time0 = img0_buf.front()->header.stamp.toSec();
      double time1 = img1_buf.front()->header.stamp.toSec();
      // 0.003s sync tolerance
      if (time0 < time1 - 0.003)
      {
          img0_buf.pop();
          printf("throw img0\n");
      }
      else if (time0 > time1 + 0.003)
      {
          img1_buf.pop();
          printf("throw img1\n");
      }
      else
      {
          time = img0_buf.front()->header.stamp.toSec();
          header = img0_buf.front()->header;
          image0 = getImageFromMsg(img0_buf.front());
          img0_buf.pop();
          image1 = getImageFromMsg(img1_buf.front());
          img1_buf.pop();
          // printf("find img0 and img1\n");
      }
  }
  // if (!image0.empty())
  //     estimator.inputImage(time, image0, image1);

}