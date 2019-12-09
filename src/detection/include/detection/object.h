#ifndef DETECTION_OBJECT_H
#define DETECTION_OBJECT_H

#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <time.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

typedef typename Eigen::Matrix<double,2,1> vector2d;
typedef typename Eigen::Matrix<double,3,1> vector3d;
typedef typename Eigen::Matrix<double,4,1> vector4d;
typedef typename Eigen::Matrix<double,6,1> vector6d;
typedef typename Eigen::Matrix<double,2,2> matrix2d;
typedef typename Eigen::Matrix<double,3,3> matrix3d;
typedef typename Eigen::Matrix<double,4,4> matrix4d;
typedef typename Eigen::Matrix<double,6,6> matrix6d;
typedef typename Eigen::Matrix<double,1,2> matrix1_2d;
typedef typename Eigen::Matrix<double,2,6> matrix2_6d;
typedef typename Eigen::Matrix<double,6,2> matrix6_2d;
typedef typename Eigen::Matrix<double,3,6> matrix3_6d;
typedef typename Eigen::Matrix<double,6,3> matrix6_3d;
typedef typename Eigen::MatrixXd matrixXd;

const float PED_WIDTH  = 0.6;
const float PED_HEIGHT = 1.7;
const float ALPHA_X = 0.8;
const float ALPHA_Y = 0.75;
const float MAX_ACC = 2.0;

typedef enum ObjectType{ VEHICLE, TRUCK, BIKE, PED, BICYCLE, UNKNOWN }ObjectType;

struct Object{
    int id;
};

struct ObjectState : Object{
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

struct RadarObject : Object{
    float r;
    float theta;
    float vt;
};

struct CameraObject : Object{
    float rx;
    float ry;
    ObjectType type;
};

struct LidarObject : Object{
    float rx;
    float ry;
    float width;
};

struct ObjectInfo : Object{
    int confi_inc;
    int confi_dec;
    int confidence;
    ObjectType type;
    float width;
    ObjectInfo(ObjectType type_, float width_){
        confi_inc = confi_dec = confidence = 0;
        type  = type_;
        width = width_;
    }
    ObjectInfo(void){
        confi_inc = confi_dec = confidence = 0;
        type  = UNKNOWN;
        width = 0.5;
    }
};

struct LocalTrack : Object{
    vector6d X;  // rx ry vx vy ax ay
    matrix6d P;
    ObjectType type;
    float width;
};

#endif // DETECTION_OBJECT_H