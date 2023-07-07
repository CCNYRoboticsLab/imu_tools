#ifndef IMU_TOOLS_COMPLEMENTARY_FILTER_NODELET_H
#define IMU_TOOLS_COMPLEMENTARY_FILTER_NODELET_H

#include <nodelet/nodelet.h>

#include "imu_complementary_filter/complementary_filter_ros.h"

class ComplementaryFilterNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
    boost::shared_ptr<imu_tools::ComplementaryFilterROS> filter_;
};

#endif  // IMU_TOOLS_COMPLEMENTARY_FILTER_NODELET_H
