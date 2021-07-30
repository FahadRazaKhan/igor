#pragma once

#include <cstdio>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include <vector>
#include <queue> // std::queue
#include <deque> // std::deque
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float64.h"
#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/Bool.h"
#include <Eigen/Dense>
#include <math.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/service.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include "rosgraph_msgs/Clock.h"
#include <gram_savitzky_golay/gram_savitzky_golay.h> //gram_savitzky_golay lib
#include <boost/circular_buffer.hpp>
#include <boost/numeric/odeint.hpp> // ODE solving library
#include "BiQuad.h"


class igor_l1_control
{

    private:
        ros::NodeHandle nh_; // creating ROS NodeHandle
        ros::Subscriber sub_body_imu; // creating ROS subscriber
        ros::Subscriber sub_CoG; // creating ROS subscriber
        ros::Subscriber sub_odom; // creating ROS subscriber

        ros::Publisher  Lwheel_pub; // creating ROS publisher
        ros::Publisher  Rwheel_pub; // creating ROS publisher
        ros::Publisher  plotPublisher;


        void body_imu_callback(const sensor_msgs::Imu::ConstPtr &msg);
        void CoG_callback(const geometry_msgs::PointStamped::ConstPtr &msg);
        void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);



        
        // float Proj2(float theta_, float y_, float Pm_, float PBar_, float epsilon_);// Projection operator
        float projection_operator(float theta, float phi, float epsilon, float theta_max, float theta_min) const;// Projection operator
        // Eigen::VectorXf stateEst(Eigen::VectorXf stateEst_, Eigen::VectorXf igorState_, Eigen::Vector2f thetaHat_, float sigmaHat_, float omegaHat_, float adaptiveCntrl_);

        // Eigen::Vector2f thetaHatDot(Eigen::Vector2f thetaHat_, Eigen::VectorXf igorState_, Eigen::VectorXf X_tilda_);
        // float sigmaHatDot(float sigmaHat_, Eigen::VectorXf X_tilda_);
        // float omegaHatDot(float omegaHat_, Eigen::VectorXf X_tilda_, float adaptiveCntrl_);

        // void adaptation(Eigen::VectorXf igorState_);
        void controlInput();
        float eitaHatFn(float u, Eigen::VectorXf y);
        void lqr_controller(Eigen::VectorXf eig_vec);

        float trapezoidal_integration(float yLast, float y_dot, float dt, float &y_dot_last);
        Eigen::VectorXf V_hat_fn(Eigen::Vector2f y, Eigen::Vector2f y_tilde);
        Eigen::VectorXf y_hat_fn(float eitaHat, Eigen::Vector2f y_tilde);
        float constrain_float(float var, float max, float min);


        Eigen::VectorXf V_hat = Eigen::VectorXf(4); //V_hat
        // Eigen::VectorXf X_hat_last = Eigen::VectorXf(2); //X_hat_previous
        Eigen::VectorXf V_hat_d = Eigen::VectorXf(4); //V_hat_dot
        Eigen::VectorXf V_hat_d_last = Eigen::VectorXf(4); 
        // Eigen::VectorXf X_hat_d_last = Eigen::VectorXf(2); //X_hat_dot_previous
        Eigen::VectorXf igorState = Eigen::VectorXf(4); // Real system states
        Eigen::VectorXf Xv_hat = Eigen::VectorXf(4); // 
        Eigen::VectorXf Xg_hat = Eigen::VectorXf(6);
        Eigen::Vector2f Xu{0,0}; //= Eigen::VectorXf(2);
        Eigen::Vector2f Xu_dot{0,0}; //= Eigen::VectorXf(2);
        Eigen::Vector2f Xu_dot_last{0,0}; //= Eigen::VectorXf(2);
        Eigen::Vector2f Y_tilda{0,0}; //= Eigen::VectorXf(2); // Output error
        Eigen::Vector2f y{0,0}; //= Eigen::VectorXf(2); // Real system output
        Eigen::Vector2f y_hat{0,0}; //= Eigen::VectorXf(2); // Output prediction
        Eigen::Vector2f y_hat_dot{0,0}; //= Eigen::VectorXf(2);
        Eigen::Vector2f y_hat_dot_last{0,0};
        Eigen::MatrixXf e_y = Eigen::MatrixXf(1,1);
        
        float thetaHat = 0; // Parameter estimate
        float thetaHat2 = 0;
        float thetaHat_d = 0; // Parameter estimate rate
        float thetaHat_d_last = 0; // Parameter estimate rate previous
        int thetaGain = 1000;
        int thetaMax = 100;
        int thetaMin = -100;
        float thetaEpsilon = 0.5;

        float sigmaHat = 0; // Sigma estimate
        float sigmaHat_d = 0; // Sigma estimate rate
        float sigmaHat_d_last = 0; // Sigma estimate rate previous
        int sigmaGain = 1000;
        int sigmaMax = 40;
        int sigmaMin = -40;
        float sigmaEpsilon = 0.5;
        float omegaHat = 1;
        float omegaHat_d = 0;
        float omegaHat_d_last = 0;
        int omegaGain = 1000;
        int omegaMax = 10;
        int omegaMin = 0;
        float omegaEpsilon = 0.1;
        // float adaptiveCntrl = 0; // Adaptive control inputs
        Eigen::MatrixXf Am = Eigen::MatrixXf(4,4);
        Eigen::MatrixXf Am_tr = Eigen::MatrixXf(4,4);
        Eigen::MatrixXf Bm = Eigen::MatrixXf(4,1);
        Eigen::MatrixXf Cm = Eigen::MatrixXf(2,4);
        Eigen::MatrixXf Cm_tr = Eigen::MatrixXf(4,2);
        Eigen::MatrixXf H = Eigen::MatrixXf(4,2);
        Eigen::MatrixXf Kv = Eigen::MatrixXf(4,2);
        Eigen::MatrixXf Av = Eigen::MatrixXf(4,4);
        Eigen::MatrixXf PvInv = Eigen::MatrixXf(4,4);
        Eigen::MatrixXf Az = Eigen::MatrixXf(2,2);
        Eigen::MatrixXf Bz = Eigen::MatrixXf(2,1);
        Eigen::MatrixXf Cz = Eigen::MatrixXf(1,2);
        Eigen::MatrixXf Dz = Eigen::MatrixXf(1,1);
        Eigen::MatrixXf Bhat = Eigen::MatrixXf(4,1);
        Eigen::MatrixXf Bhat_tr = Eigen::MatrixXf(1,4);
        Eigen::MatrixXf Uz = Eigen::MatrixXf(1,1);
        // Eigen::Vector2f C{1,0};
        Eigen::MatrixXf Py = Eigen::MatrixXf::Identity(2,2);
        Eigen::Vector3d CoM_vec;
        Eigen::Matrix3d pitchRotEigen;
        Eigen::Vector3d groundPoint;
        Eigen::Vector3d rightLegTranslation;
        Eigen::Vector3d leftLegTranslation;
        Eigen::Vector3d CoM_line;
        Eigen::Vector2f trig_vec; // declaring 2X1 Eigen vector of datatype float
        Eigen::MatrixXf pos_vec = Eigen::MatrixXf(1,2);
        Eigen::MatrixXf vel_vec = Eigen::MatrixXf(1,2);
        Eigen::MatrixXf Kg = Eigen::MatrixXf(1,2);
        Eigen::VectorXf refState = Eigen::VectorXf(2); // reference states
        Eigen::Vector2f filterInput{0,0};
        Eigen::MatrixXf k_r = Eigen::MatrixXf(1,4); // declaring 1X6 Eigen matrix of datatype float
        Eigen::MatrixXf k_l = Eigen::MatrixXf(1,4); // declaring 1X6 Eigen matrix of datatype float

        Eigen::Vector2f b{0,-0.4};

        // std::vector<decltype(thetaHat)::value_type> FilterOut;
       

        geometry_msgs::Quaternion  igor_orient;  // Quaternion type variable
        geometry_msgs::Vector3 igor_angul_vel; // Vector3 type variable
        geometry_msgs::Vector3 igor_linear_accl;
        geometry_msgs::Point CoG_Position;
        geometry_msgs::TransformStamped leftLegTransformStamped;
        geometry_msgs::TransformStamped rightLegTransformStamped;
        geometry_msgs::PoseWithCovariance igor_pose;
        geometry_msgs::TwistWithCovariance igor_twist;
        geometry_msgs::Point igor_position;
        geometry_msgs::Vector3 igor_linear_vel; 
        

        std_msgs::Float64 trq_r;
        std_msgs::Float64 trq_l;
        std_msgs::Float32MultiArray PlotingVector;

        tf::Quaternion quat;
        tf::Matrix3x3 pitchRotation;
        
        tf2_ros::Buffer leftLegTfBuffer;
        tf2_ros::TransformListener leftLegTfListener{leftLegTfBuffer};
        tf2_ros::Buffer rightLegTfBuffer;
        tf2_ros::TransformListener rightLegTfListener{rightLegTfBuffer};


        double roll = 0; 
        double pitch = 0; 
        double yaw = 0;

        float pitch_vel = 0;
        float yaw_vel = 0;
        float leanAngle = 0; 
        float CoM_height = 0;
        float CoG_PitchAngle_filtered = 0;
        float eitaHat = 0;
        float alpha = 25;
        float Xg_Norm = 0;
        float L1_Input = 0;


        float igor_pos_x = 0;
        float igor_pos_y = 0;
        float igor_vel_x = 0;
        float igor_vel_y = 0;
        float igor_center_position = 0;
        float igor_center_vel = 0;
        float dt = 0.002;

        
        
        BiQuad bq1{0.0001416, 0.0002832, 0.0001416, -1.966, 0.967};
        BiQuad bq2{0.0001416, 0.0002832, 0.0001416, -1.966, 0.967};
        BiQuad bq3{0.0, 0.09516, 0.0, -0.9048, 0.0};


    public:

        igor_l1_control(ros::NodeHandle* nodehandle); // constructor
        ~igor_l1_control(); // destructor

        // Window size is 2*m+1
        const unsigned int m1 = 12;
        const unsigned int m2 = 5;
        const unsigned int m3 = 50;
        // Polynomial Order
        const unsigned int n1 = 0;
        const unsigned int n2 = 1;
        const unsigned int n3 = 2;
        // Initial Point Smoothing (ie evaluate polynomial at first point in the window)
        // Points are defined in range [-m;m]
        const int t1 = m1;
        const int t2 = m2;
        const int t3 = m3;
        // Derivate? 0: no derivation, 1: first derivative...
        const unsigned int d = 0;
        //double result;
        gram_sg::SavitzkyGolayFilterConfig sg_conf1{m1,t1,n1,d,0.002}; // filter configuration
        gram_sg::SavitzkyGolayFilterConfig sg_conf2{m2,t2,n2,1,0.002}; // filter configuration
        gram_sg::SavitzkyGolayFilterConfig sg_conf3{m3,t3,n3,2,0.002}; // filter configuration
        
        gram_sg::SavitzkyGolayFilter pitchFilt{sg_conf1};
        gram_sg::SavitzkyGolayFilter pitch_vel_filt{sg_conf1};
        gram_sg::SavitzkyGolayFilter yaw_vel_filt{sg_conf1};
        gram_sg::SavitzkyGolayFilter doubleDerivativeFilt{sg_conf3};
        gram_sg::SavitzkyGolayFilter derivativeFilt{sg_conf2};

        boost::circular_buffer<double> pitchVelVector {boost::circular_buffer<double>((2*m1+1),0)};
        boost::circular_buffer<double> yawVelVector {boost::circular_buffer<double>((2*m1+1),0)};
        boost::circular_buffer<double> leanAngleVector {boost::circular_buffer<double>((2*m1+1),0)};
        boost::circular_buffer<double> doubleDerivativeVector {boost::circular_buffer<double>((2*m3+1),0)};
        boost::circular_buffer<double> derivativeVector {boost::circular_buffer<double>((2*m2+1),0)};



};