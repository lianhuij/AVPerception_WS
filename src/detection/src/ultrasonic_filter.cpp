#include <ros/ros.h>
#include <raw_data/Ultrasonic.h>
#include <detection/ultrasonic_filter.h>

extern ros::Publisher left_ultrasonic_pub, right_ultrasonic_pub;
extern UltrasonicFilter left_ultrasonic;
extern UltrasonicFilter right_ultrasonic;
extern float KF_Q;
extern float KF_R;
extern float MAX_RANGE;
extern int MAX_LOST;

UltrasonicFilter::UltrasonicFilter(void)
{
    init_P = 5;
    for(int i=0; i<4; ++i){
      track_flag[i] = false;
      lost[i] = 0;
    }
}

UltrasonicFilter::~UltrasonicFilter() { }

void UltrasonicFilter::KF(const raw_data::Ultrasonic& input)
{
    raw_data::Ultrasonic raw;
    for(int i=0; i<4; ++i){
      raw.probe[i] = input.probe[i];
    }
    time_stamp = input.header.stamp;

    //确定跟踪标志位
    for(int i=0; i<4; ++i){
      if(track_flag[i]){//1
        if(raw.probe[i] < MAX_RANGE){
          lost[i] = 0;
          continue;
        }else{
          lost[i]++;
          if(lost[i] >= MAX_LOST){
            X[i] = MAX_RANGE;
            track_flag[i] = false;
            lost[i] = 0;
          }else{
            raw.probe[i] = X[i];  //容错范围内，沿用上一时刻检测值
          }
        }
      }else{//1
        if(raw.probe[i] < MAX_RANGE){ //开启跟踪器
          track_flag[i] = true;
          X[i] = raw.probe[i];
          P[i] = init_P;
        }else{
          X[i] = MAX_RANGE;
        }
      }
    }
    
    Predict();
    Update(raw);
    PubUltrasonic();
}

void UltrasonicFilter::Predict(void)
{
    for (int i=0; i<4; ++i)
    {
        if(track_flag[i]){
          P[i] = P[i] + KF_Q;
        }
    }
}

void UltrasonicFilter::Update(const raw_data::Ultrasonic& raw)
{
    float K[4];
    for(int i=0; i<4; ++i)
    {
        if(track_flag[i]){
          K[i] = P[i] / (P[i] + KF_R);
          X[i] = X[i] + K[i] * (raw.probe[i] - X[i]);
          P[i] = (1 - K[i]) * P[i];
        }
    }
}

void UltrasonicFilter::PubUltrasonic(void)
{
    raw_data::Ultrasonic probe;
    for(int i=0; i<4; ++i){
      probe.probe[i] = X[i];
      if(probe.probe[i] > MAX_RANGE){
        probe.probe[i] = MAX_RANGE;
      }
    }
    probe.header.stamp = time_stamp;
    if(this == &left_ultrasonic){
      left_ultrasonic_pub.publish(probe);
      return;
    }
    if(this == &right_ultrasonic){
      right_ultrasonic_pub.publish(probe);
      return;
    }
}