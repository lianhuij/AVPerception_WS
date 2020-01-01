#include <ros/ros.h>
#include <raw_data/Ultrasonic.h>
#include <detection/ultrasonic_filter.h>

extern ros::Publisher left_ultrasonic_pub, right_ultrasonic_pub;
extern UltrasonicFilter* left_ultrasonic_ptr;
extern UltrasonicFilter* right_ultrasonic_ptr;

UltrasonicFilter::UltrasonicFilter(void)
{
    X[0] = -1;   //for initialize
    init_P = 5;
    Q = 0.5;
    R = 1;
}

UltrasonicFilter::~UltrasonicFilter() { }

void UltrasonicFilter::KF(const raw_data::Ultrasonic& input)
{
    raw_data::Ultrasonic raw;
    for(int i=0; i<4; ++i){
      raw.probe[i] = input.probe[i];
    }
    time_stamp = ros::Time::now();

    if (X[0] < 0)
    {
      InitTrack(raw);
    }
    else
    {
      Predict();
      Update(raw);
    }
    PubUltrasonic();
}

void UltrasonicFilter::InitTrack(const raw_data::Ultrasonic& raw)
{
    for(int i=0; i<4; ++i){
      X[i] = raw.probe[i];
      P[i] = init_P;
    }
}

void UltrasonicFilter::Predict(void)
{
    for (int i=0; i<4; ++i)
    {
        P[i] = P[i] + Q;
    }
}

void UltrasonicFilter::Update(const raw_data::Ultrasonic& raw)
{
    float K[4];
    for(int i=0; i<4; ++i)
    {
        K[i] = P[i] / (P[i] + R);
        X[i] = X[i] + K[i] * (raw.probe[i] - X[i]);
        P[i] = (1 - K[i]) * P[i];
    }
}

void UltrasonicFilter::PubUltrasonic(void)
{
    raw_data::Ultrasonic probe;
    for(int i=0; i<4; ++i){
      probe.probe[i] = X[i];
    }
    probe.header.stamp = time_stamp;
    if(this == left_ultrasonic_ptr){
      left_ultrasonic_pub.publish(probe);
      return;
    }
    if(this == right_ultrasonic_ptr){
      right_ultrasonic_pub.publish(probe);
      return;
    }
}