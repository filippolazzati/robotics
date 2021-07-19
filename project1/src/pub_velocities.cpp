#define _USE_MATH_DEFINES
#include <cmath>
#include <boost/shared_ptr.hpp>
#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "geometry_msgs/TwistStamped.h"
#include "robotics_hw1/MotorSpeed.h"


typedef message_filters::sync_policies::ApproximateTime<robotics_hw1::MotorSpeed,
            robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed> SyncPolicy;

/*
    This node listens to the four topics where the wheels' velocities are published,
    synchronizing the messages with a message filter (with an approximate time policy),
    extimates the linear and the angular velocity of the robot and publishes them in
    the topic /velocities as TwistStamped messages.
*/

class velocity_calculator {
    private :

        static constexpr double APPARENT_BASELINE = 0.9; //value in meters
        static constexpr double WHEEL_RADIUS = 0.1575; //value in meters
        static constexpr double GEAR_RATIO = 0.0223; // ratio between the angular velocity of the wheels and the angular velocity of the motors

        ros::NodeHandle n;

        message_filters::Subscriber<robotics_hw1::MotorSpeed> subfl;
        message_filters::Subscriber<robotics_hw1::MotorSpeed> subfr;
        message_filters::Subscriber<robotics_hw1::MotorSpeed> subrl;
        message_filters::Subscriber<robotics_hw1::MotorSpeed> subrr;
        boost::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

        ros::Publisher pub;

    public:

    void calculate_velocities(const robotics_hw1::MotorSpeed::ConstPtr& msgfl,
                              const robotics_hw1::MotorSpeed::ConstPtr& msgfr,
                              const robotics_hw1::MotorSpeed::ConstPtr& msgrl,
                              const robotics_hw1::MotorSpeed::ConstPtr& msgrr) {   
        // callback of the message filter synchronizer: extimates linear and angular
        // velocity of the robot from the four motors' speeds

        // linear velocity of left wheels
        double vl = - ((msgfl->rpm + msgrl->rpm) / 2) * (M_PI/30) * GEAR_RATIO * WHEEL_RADIUS;
        // linear velocity of right wheels
        double vr =  ((msgfr->rpm + msgrr->rpm) / 2) * (M_PI/30) * GEAR_RATIO * WHEEL_RADIUS;

        // linear velocity of the robot
        double vx = (vl+vr)/2;
        // angular velocity of the robot
        double wz = (-vl+vr)/APPARENT_BASELINE;

        geometry_msgs::TwistStamped msg_vel;
        msg_vel.header.seq = msgfl->header.seq;
        msg_vel.header.stamp = msgfl->header.stamp; // the stamp of the messages refers to the time the sensor data arrived,
                                                    // not to the time of the messages' publication
        msg_vel.header.frame_id = "base_link"; // the frame id which the velocities refer to
        msg_vel.twist.linear.x = vx;
        msg_vel.twist.linear.y = 0.0;
        msg_vel.twist.linear.z = 0.0;
        msg_vel.twist.angular.x = 0.0;
        msg_vel.twist.angular.y = 0.0;
        msg_vel.twist.angular.z = wz;
        pub.publish(msg_vel);
    }

    velocity_calculator() {

        // setting up the syncronization of the four topics with the motor speed,
        // using a message filter with approximate time policy
        this->subfl.subscribe(this->n, "motor_speed_fl", 10);
        this->subfr.subscribe(this->n, "motor_speed_fr", 10);
        this->subrl.subscribe(this->n, "motor_speed_rl", 10);
        this->subrr.subscribe(this->n, "motor_speed_rr", 10);
        this->sync.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), subfl, subfr,subrl,subrr));
        this->sync->registerCallback(boost::bind(&velocity_calculator::calculate_velocities, this, _1, _2, _3, _4));

        //publisher of the linear and angular velocity of the robot, computed from the motor speeds
        this->pub = n.advertise<geometry_msgs::TwistStamped>("/velocities", 1);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pub_velocities");
    velocity_calculator velocity_calculator;
    ros::spin();
    return 0;
}
