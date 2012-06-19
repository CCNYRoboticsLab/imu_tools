#ifndef IMU_FILTER_IMU_FILTER_NODELET_H
#define IMU_FILTER_IMU_FILTER_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "imu_filter/imu_filter.h"

class ImuFilterNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
    ImuFilter * filter_;  // FIXME: change to smart pointer
};

#endif // IMU_FILTER_IMU_FILTER_NODELET_H
