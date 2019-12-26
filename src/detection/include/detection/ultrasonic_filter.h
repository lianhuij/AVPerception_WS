#ifndef DETECTION_ULTRASONIC_FILTER_H
#define DETECTION_ULTRASONIC_FILTER_H

#include <ros/ros.h>
#include <raw_data/Ultrasonic.h>

class UltrasonicFilter
{
public:
    UltrasonicFilter(void);
    ~UltrasonicFilter();

    void KF(const raw_data::Ultrasonic& input);
    void InitTrack(const raw_data::Ultrasonic& raw);
    void Predict(void);
    void Update(const raw_data::Ultrasonic& raw);
    void PubUltrasonic(void);

private:
    float X[4];  
    float P[4];
    float init_P;
    float Q;
    float R;
};

#endif // DETECTION_ULTRASONIC_FILTER_H