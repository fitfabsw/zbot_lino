#include "hcsr04.h"


HCSR04::HCSR04(char* link_name)
{
    // range_msg_.header.frame_id = micro_ros_string_utilities_set(range_msg_.header.frame_id, "sonic1_link");
    range_msg_.header.frame_id = micro_ros_string_utilities_set(range_msg_.header.frame_id, link_name);
}


bool HCSR04::init(int trig, int echo)
{
    bool sensor_ok = startSensor(trig, echo);
    return sensor_ok;
}


sensor_msgs__msg__Range HCSR04::getData()
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    pinMode(echoPin, INPUT);
    duration = pulseIn(echoPin, HIGH);
    distance_m = (duration/2) * 0.01 / 29.1;     // Divide by 29.1 or multiply by 0.0343

    // range_msg_.radiation_type = Range.ULTRASOUND;
    range_msg_.radiation_type = 0;
    range_msg_.field_of_view = 0.6108; // 35 degree
    range_msg_.min_range = 0.05;
    range_msg_.max_range = 0.5;
    range_msg_.range = distance_m;

    delayMicroseconds(2000);

    return range_msg_;
}

bool HCSR04::startSensor(int trig, int echo)
{
    // trigPin = 22;    // Trigger
    // echoPin = 23;    // Echo
    trigPin = trig;
    echoPin = echo;
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    return true;
}
