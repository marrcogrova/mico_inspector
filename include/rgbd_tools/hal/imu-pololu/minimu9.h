#pragma once

// TODO: I'd rather have the minimu9 and imu class not know anything about
// floating point vectors and calibration.  Maybe just provide floating
// point scaling factors so that higher-level code can make sense of the
// raw vectors.

#include <rgbd_tools/hal/imu-pololu/imu.h>
#include <rgbd_tools/hal/imu-pololu/lsm303.h>
#include <rgbd_tools/hal/imu-pololu/l3g.h>
#include <rgbd_tools/hal/imu-pololu/lsm6.h>
#include <rgbd_tools/hal/imu-pololu/lis3mdl.h>
#include <rgbd_tools/hal/imu-pololu/sensor_set.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <wordexp.h>

namespace minimu9
{
  // Represents the sensors of the MinIMU-9 and how to communicate with them.
  struct comm_config {
    lsm303::comm_config lsm303;
    l3g::comm_config l3g;
    lis3mdl::comm_config lis3mdl;
    lsm6::comm_config lsm6;
  };

  comm_config auto_detect(const std::string & i2c_bus_name);

  sensor_set config_sensor_set(const comm_config &);

  comm_config disable_redundant_sensors(const comm_config &, const sensor_set &);

  class handle : public imu {
  public:
    void open(const comm_config &);

    comm_config config;
    lsm6::handle lsm6;
    lis3mdl::handle lis3mdl;
    lsm303::handle lsm303;
    l3g::handle l3g;

    virtual void read_acc_raw();
    virtual void read_mag_raw();
    virtual void read_gyro_raw();

    virtual float get_acc_scale() const;
    virtual float get_gyro_scale() const;

    virtual vector read_acc();
    virtual vector read_mag();
    virtual vector read_gyro();

    virtual void enable();
    virtual void load_calibration();
    virtual void measure_offsets();
  };
}
