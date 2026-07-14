/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

// SYSTEM
#include <atomic>

#ifdef HAS_CPUID
#include <cpuid.h>
#endif

#include <car_status_msgs/msg/asfsm_state.hpp>
#include <car_status_msgs/msg/asfsm_state_enum.hpp>
#include <ctime>
#include <fstream>
#include <future>
#include <iomanip>
#include <ios>
#include <iostream>
#include <mutex>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/times.h>
#include <thread>

template<typename T>
std::string to_string_with_precision(const T a_value, const int n = 6) {
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

// BOOST
#include <boost/format.hpp>

// PCL
#define PCL_NO_PRECOMPILE

// DLIO
#include <nano_gicp/nano_gicp.h>

namespace dlio {
class OdomNode;
class MapNode;

struct EIGEN_ALIGN16 Point {
    Point() : data{ 0.f, 0.f, 0.f, 1.f } {}

    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    double timestamp;
    float range;
    float azimuth;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace dlio

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(dlio::Point,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (uint16_t, ring, ring)
                                 (double, timestamp, timestamp)
                                 (float, range, range)
                                 (float, azimuth, azimuth))
// clang-format on

typedef dlio::Point PointType;
