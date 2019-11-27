#ifndef DETECTION_OBJECT_H
#define DETECTION_OBJECT_H

#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <time.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

typedef typename Eigen::Matrix<double,3,1> vector3d;
typedef typename Eigen::Matrix<double,6,1> vector6d;
typedef typename Eigen::Matrix<double,3,3> matrix3d;
typedef typename Eigen::Matrix<double,3,6> matrix3_6d;
typedef typename Eigen::Matrix<double,6,3> matrix6_3d;
typedef typename Eigen::Matrix<double,6,6> matrix6d;

const float ped_width  = 0.6;
const float ped_height = 1.7;

struct Object
{
    uint64_t id;
};

struct BoxObject : Object
{
    float rx;
    float ry;
    float vx;
    float vy;
    float ax;
    float ay;

    float rx_cov;
    float ry_cov;
    float vx_cov;
    float vy_cov;
};

struct RadarObject : Object
{
    float r;
    float theta;
    float vt;
};

struct LidarPoint
{
    float rx;
    float ry;
};

struct TrackCount
{
    int confi_inc;
    int confi_dec;
    int confidence;
};

#endif // DETECTION_OBJECT_H