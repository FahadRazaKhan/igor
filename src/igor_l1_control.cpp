#include "igor_l1_control.h"



igor_l1_control::igor_l1_control(ros::NodeHandle* nodehandle):nh_(*nodehandle) //Constructor
{

    sub_body_imu = nh_.subscribe<sensor_msgs::Imu>("/igor/body_imu/data",1, &igor_l1_control::body_imu_callback,this);
    sub_CoG = nh_.subscribe<geometry_msgs::PointStamped>("/cog/robot",1, &igor_l1_control::CoG_callback,this);
    sub_odom = nh_.subscribe<nav_msgs::Odometry>("/igor/odom",1, &igor_l1_control::odom_callback,this,ros::TransportHints().tcpNoDelay());

    Lwheel_pub = nh_.advertise<std_msgs::Float64>( "/igor/L_wheel_joint_effort_controller/command", 1 );
    Rwheel_pub = nh_.advertise<std_msgs::Float64>( "/igor/R_wheel_joint_effort_controller/command", 1 );
    plotPublisher = nh_.advertise<std_msgs::Float32MultiArray>( "/igor/plotingVec", 5);
    
    // Vector Initialization 
    // V_hat(0) = 0;
    // V_hat(1) = 0;
    // V_hat(2) = 0;
    // V_hat(3) = 0;
    // X_hat(4) = 0;
    // X_hat(5) = 0;

    // X_hat_last(0) = 0;
    // X_hat_last(1) = 0;
    // X_hat_last(2) = 0;
    // X_hat_last(3) = 0;
    // X_hat_last(4) = 0;
    // X_hat_last(5) = 0;

    // V_hat_d(0) = 0;
    // V_hat_d(1) = 0;
    // V_hat_d(2) = 0;
    // V_hat_d(3) = 0;
    // X_hat_d(4) = 0;
    // X_hat_d(5) = 0;

    // V_hat_d_last(0) = 0;
    // V_hat_d_last(1) = 0;
    // V_hat_d_last(2) = 0;
    // V_hat_d_last(3) = 0;
    // X_hat_d_last(4) = 0;
    // X_hat_d_last(5) = 0;

    // Xv_hat(0) = 0;
    // Xv_hat(1) = 0;
    // Xv_hat(2) = 0;
    // Xv_hat(3) = 0;

    // Xg_hat(0) = 0;
    // Xg_hat(1) = 0;
    // Xg_hat(2) = 0;
    // Xg_hat(3) = 0;
    // Xg_hat(4) = 0;
    // Xg_hat(5) = 0;

    // e_y(0) = 0;
    // Uz(0) = 0;

    // X_tilda(0) = 0;
    // X_tilda(1) = 0;
    // X_tilda(2) = 0;
    // X_tilda(3) = 0;
    // X_tilda(4) = 0;
    // X_tilda(5) = 0;

    igorState(0) = 0;
    igorState(1) = 0;
    igorState(2) = 0;
    igorState(3) = 0;
    igorState(4) = 0;
    igorState(5) = 0;

    Am(0,0) = 0;
    Am(0,1) = 1;
    // Am(0,2) = 1;
    // Am(0,3) = 0;
    // Am(0,4) = 0;
    // Am(0,5) = 0;
    Am(1,0) = -150;
    Am(1,1) = -430;
    // Am(1,2) = 0;
    // Am(1,3) = 1;
    // Am(1,4) = 1;
    // Am(1,5) = 0;
    // Am(2,0) = -0.6604;
    // Am(2,1) = 13.5280;
    // Am(2,2) = -2.5701;
    // Am(2,3) = 1.2709;
    // Am(2,4) = 0;
    // Am(2,5) = 1;
    // Am(3,0) = 0.8462;
    // Am(3,1) = -21.7878;
    // Am(3,2) = 3.2931;
    // Am(3,3) = -1.6694;
    // Am(3,4) = 0;
    // Am(3,5) = -26.4754;
    // Am(4,0) = 0;
    // Am(4,1) = -179.9585;
    // Am(4,2) = 0;
    // Am(4,3) = 0;
    // Am(4,4) = -47.8622;
    // Am(4,5) = 0;
    // Am(5,0) = -189.2728;
    // Am(5,1) = 0;
    // Am(5,2) = 219.8499;
    // Am(5,3) = 539.5515;
    // Am(5,4) = 0;
    // Am(5,5) = 52.0397; 

    Am_Inv = Am.completeOrthogonalDecomposition().pseudoInverse();

    // Bm(0,0) = 0;
    // Bm(0,1) = 0;
    // Bm(1,0) = 0;
    // Bm(1,1) = 0;
    // Bm(2,0) = 0.2696;
    // Bm(2,1) = 0;
    // Bm(3,0) = -0.3454;//9.9680;
    // Bm(3,1) = 1;//9.9680;
    // Bm(4,0) = 1;//18.6288;
    // Bm(4,1) = -1;//-18.6288;
    // Bm(5,0) = -1;//-19.5930;
    // Bm(5,1) = -1;//-19.5930; 

    A(0,0) = 0;
    A(0,1) = 0;
    A(0,2) = 1;
    A(0,3) = 0;
    A(1,0) = 0;
    A(1,1) = 0;
    A(1,2) = 0;
    A(1,3) = 1;
    A(2,0) = 0;
    A(2,1) = 8.2291;
    A(2,2) = 0;
    A(2,3) = 0;
    A(3,0) = 0;
    A(3,1) = -13.8686;
    A(3,2) = 0;
    A(3,3) = 0;

    B(0,0) = 0;
    B(1,0) = 0;
    B(2,0) = 0.2701;
    B(3,0) = -0.3154;



    // Cm(0,0) = 1; 
    // Cm(0,1) = 0;
    // Cm(0,2) = 0;
    // Cm(0,3) = 0;
    // Cm(1,0) = 0;
    // Cm(1,1) = 1;
    // Cm(1,2) = 0;
    // Cm(1,3) = 0;

    // Cm_tr = Cm.transpose();
    

    // H(0,0) = 0.3872;
    // H(0,1) = -0.4871;
    // H(1,0) = -0.4871;
    // H(1,1) = 0.6128;
    // H(2,0) = -2.8743;
    // H(2,1) = 3.6161;
    // H(3,0) = 4.6631;
    // H(3,1) = -5.8665;

    // Av(0,0) = -19.2679;
    // Av(0,1) = -12.8895;
    // Av(0,2) = 0.6128;
    // Av(0,3) = 0.4871;
    // Av(1,0) = -12.9153;
    // Av(1,1) = -13.2333;
    // Av(1,2) = 0.4871;
    // Av(1,3) = 0.3872;
    // Av(2,0) = 450.7897;
    // Av(2,1) = 356.6743;
    // Av(2,2) = 0.3042;
    // Av(2,3) = -2.3452;
    // Av(3,0) = -867.9724;
    // Av(3,1) = -686.8016;
    // Av(3,2) = -1.3700;
    // Av(3,3) = 4.1971;

    // Av = 1*Av;

    // PvInv(0,0) = 0.3401;
    // PvInv(0,1) = -0.4315;
    // PvInv(0,2) = 0.0019;
    // PvInv(0,3) = -0.0140;
    // PvInv(1,0) = -0.4315;
    // PvInv(1,1) = 0.5488;
    // PvInv(1,2) = -0.0167;
    // PvInv(1,3) = 0.0450;
    // PvInv(2,0) = 0.0019;
    // PvInv(2,1) = -0.0167;
    // PvInv(2,2) = 0.1709;
    // PvInv(2,3) = -0.3265;
    // PvInv(3,0) = -0.0140;
    // PvInv(3,1) = 0.0450;
    // PvInv(3,2) = -0.3265;
    // PvInv(3,3) = 0.6250;

    


    // Az(0,0) = -2.2759;
    // Az(0,1) = -1.0002;
    // Az(1,0) = 0.8862;
    // Az(1,1) = -4.2241;

    // Bz(0,0) = 0;
    // Bz(1,0) = 0.4382;

    // Cz(0,0) = 2.3806;
    // Cz(0,1) = 0;
    
    // Dz(0,0) = 0;

    // Bhat(0,0) = -0.5948;
    // Bhat(1,0) = 0.7483;
    // Bhat(2,0) = 4.4156;
    // Bhat(3,0) = -7.1635;

    // Bhat_tr = Bhat.transpose();

    // Kv(0,0) = -19.2679;
    // Kv(0,1) = -12.8895;
    // Kv(1,0) = -12.9153;
    // Kv(1,1) = -13.2333;
    // Kv(2,0) = 451.4501;
    // Kv(2,1) = 343.1463;
    // Kv(3,0) = -868.8186;
    // Kv(3,1) = -665.0138;

    // Kv = 1*Kv;


    P(0,0) = 1.6089;
    P(0,1) = 0.0033;
    P(1,0) = 0.0033;
    P(1,1) = 0.0012;
   
   

    // Reference states
    refState(0) = 0; // Center Position 
    refState(1) = 0; // Yaw
    refState(2) = 0; // Pitch
    refState(3) = 0; // Linear Velocity
    refState(4) = 0; // Yaw rate
    refState(5) = 0; // Pitch rate
 

    // Am_Inv = Am.completeOrthogonalDecomposition().pseudoInverse();
    // Kg = (C*Am_Inv*Bm).completeOrthogonalDecomposition().pseudoInverse(); // Feedforward gain
    // Kg = -1*Kg;
    // Kg(0,0) = 1;
    // Kg(0,1) = 2;
    Kg = -375;

    PlotingVector.data.resize(4); // Resizing std::msg array

    k_r(0,0)= k_l(0,0) = 4*(-0.7071); // Forward position gain -ve
    k_r(0,1)= 2*(0.7071); // Yaw gain +ve
    k_r(0,2)= k_l(0,2) = 1.2*(-16.2331); // Pitch gain -ve
    k_r(0,3)= k_l(0,3) = (-4.8849); // Forward speed gain -ve
    k_r(0,4)= (0.4032); // Yaw speed gain +ve
    k_r(0,5)= k_l(0,5)= 1.5*(-3.1893); // Pitch speed gain -ve
    k_l(0,1)= -1*k_r(0,1);
    k_l(0,4)= -1*k_r(0,4);

    

    

}// End of Constructor



void igor_l1_control::body_imu_callback(const sensor_msgs::Imu::ConstPtr &msg){

    igor_orient = msg->orientation; // a geometery_msg quaternion
    igor_angul_vel = msg->angular_velocity;
    igor_linear_accl = msg->linear_acceleration;

    pitch_vel = floorf(igor_angul_vel.y*10000)/10000; // round off data upto 4 decimal points
    yaw_vel = floorf(igor_angul_vel.z*10000)/10000;
        
    tf::quaternionMsgToTF(igor_orient, quat); // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaternion
    
    quat.normalize(); // normalize the quaternion in case it is not normalized
    
    //the tf::Quaternion has a method to acess roll pitch and yaw
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    

    pitchVelVector.push_back(pitch_vel);
   
    
    
    yawVelVector.push_back(yaw_vel);
    
    
    
    igorState(1) = floorf(yaw*10000)/10000;// Yaw angle
    igorState(4) = yaw_vel_filt.filter(yawVelVector); // Yaw velocity
    igorState(5) = pitch_vel_filt.filter(pitchVelVector); // Pitch Velocity

    // y(1) = pitch_vel_filt.filter(pitchVelVector); // Pitch Velocity

    



}// End of imu_callback

void igor_l1_control::CoG_callback(const geometry_msgs::PointStamped::ConstPtr &msg){

    CoG_Position = msg->point;
    CoM_vec << CoG_Position.x, CoG_Position.y, CoG_Position.z;
    pitchRotation.setRPY(0,pitch,0); // Setting Pitch rotation matrix
    tf::matrixTFToEigen(pitchRotation, pitchRotEigen); // Converting tf matrix to Eigen matrix
  

    try
    { 
        leftLegTransformStamped = leftLegTfBuffer.lookupTransform("base_link", "L_wheelActuator" , ros::Time(0));
        rightLegTransformStamped = rightLegTfBuffer.lookupTransform("base_link", "R_wheelActuator" , ros::Time(0));
    }
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("%s",ex.what());
    }

    leftLegTranslation << leftLegTransformStamped.transform.translation.x, leftLegTransformStamped.transform.translation.y, leftLegTransformStamped.transform.translation.z;
    rightLegTranslation << rightLegTransformStamped.transform.translation.x, rightLegTransformStamped.transform.translation.y, rightLegTransformStamped.transform.translation.z;
    // Find the mean of the two legs' translation vectors in base_link frame
    groundPoint = 0.5*(leftLegTranslation+rightLegTranslation);
    //Get the vector starting from the "ground point" and ending at the position of the current center of mass
    CoM_line = CoM_vec - groundPoint;
    // Rotate it according to the current pitch angle of Igor
    CoM_line = pitchRotEigen * CoM_line; 
    CoM_height =  CoM_line.norm();
    // Lean/Pitch angle of CoM from the wheel base 
    leanAngle = atan2(CoM_line.x(), CoM_line.z());

    //std::cout<<"Lean angle: " << std::endl << leanAngle << std::endl;
    
    /**#####################################################**/
    

    
    leanAngleVector.push_back(leanAngle);
    

    CoG_PitchAngle_filtered = pitchFilt.filter(leanAngleVector);
 
 


    // ROS_INFO("CoG angle: %f", CoG_PitchAngle_filtered);
    // std::cout << "Kg:" << std::endl << Kg << std::endl;
    // std::cout << "Bm:" << std::endl << Bm << std::endl;
    // std::cout << "Am:" << std::endl << Am << std::endl;
    // std::cout << "Cm:" << std::endl << Cm << std::endl;
    // std::cout << "H:" << std::endl << H << std::endl;
    // std::cout << "Av:" << std::endl << Av << std::endl;
    // std::cout << "Py:" << std::endl << Py << std::endl;
    // std::cout << "PvInv:" << std::endl << PvInv << std::endl;
    // std::cout << "Az:" << std::endl << Az << std::endl;
    // std::cout << "Bz:" << std::endl << Bz << std::endl;
    // std::cout << "Cz:" << std::endl << Cz << std::endl;
    // std::cout << "Dz:" << std::endl << Dz << std::endl;
    // std::cout << "Bhat:" << std::endl << Bhat << std::endl;
    // std::cout << "Kv:" << std::endl << Kv << std::endl;
    // std::cout << "e_y:" << std::endl << e_y << std::endl;
    // std::cout << "Uz:" << std::endl << Uz << std::endl;
    // std::cout << "Xg_hat:" << std::endl << Xg_hat << std::endl;
    



    igorState(2) = CoG_PitchAngle_filtered;
    // y(0) = CoG_PitchAngle_filtered;

}// End of CoG_callback


void igor_l1_control::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{

    igor_pose = msg->pose; // igor pose
    igor_twist = msg->twist; // igor twist
    igor_position = igor_pose.pose.position; // igor linear position
    //igor_orient = igor_pose.pose.orientation;
    igor_linear_vel = igor_twist.twist.linear; // igor linear velocity
    //igor_angul_vel = igor_twist.twist.angular; 



    igor_pos_x = igor_position.x;
    igor_pos_x = floorf(igor_pos_x*1000)/1000;
    igor_vel_x = igor_linear_vel.x;  

    igor_pos_y = (igor_position.y);
    igor_pos_y = floorf(igor_pos_y*1000)/1000;
    igor_vel_y = igor_linear_vel.y;
    
    
    trig_vec(0) = cos(floorf(yaw*1000)/1000);
    trig_vec(1) = sin(floorf(yaw*1000)/1000);
    pos_vec(0,0) = igor_pos_x;
    pos_vec(0,1) = igor_pos_y;
    vel_vec(0,0) = igor_vel_x;
    vel_vec(0,1) = igor_vel_y;

    igor_center_position = (pos_vec*trig_vec).value();
    igor_center_vel = (vel_vec*trig_vec).value();
   

    igorState(0) = igor_center_position;
    igorState(3) = floorf(igor_center_vel*1000)/1000;

    //ROS_INFO("Igor Position: %f",igor_center_position);

    // PlotingVector.data[2] = -y(0);
    // PlotingVector.data[3] = y(1);

    // this->adaptation(y);
    // this->lqr_controller(igorState);
    // this->L1ControlInput(y);
    this->augmented_controller(igorState);
    plotPublisher.publish(PlotingVector);

}// End of odom_callback

Eigen::VectorXf igor_l1_control::stateEst(Eigen::VectorXf stateEst_, Eigen::VectorXf igorState_, Eigen::Vector2f thetaHat_, float sigmaHat_,float omegaHat_, float adaptiveCntrl_){

    //ROS_INFO("In stateEst");
    float igorStateNorm = igorState_.lpNorm<Eigen::Infinity>(); // Infinity Norm
    X_hat_d = (Am*stateEst_) + b*((omegaHat_*adaptiveCntrl_)+ thetaHat_.transpose()*igorState_ + sigmaHat_);

    X_hat(0) = this->trapezoidal_integration(X_hat(0),X_hat_d(0),dt,X_hat_d_last(0));
    X_hat(1) = this->trapezoidal_integration(X_hat(1),X_hat_d(1),dt,X_hat_d_last(1));
    return X_hat;

} // End of State Predictor

float igor_l1_control::L1ControlInput(Eigen::VectorXf y){

    ROS_INFO("In L1ControlInput");

    X_tilda = X_hat-y;
   

    X_hat = this->stateEst(X_hat, y, thetaHat, sigmaHat, omegaHat,L1_Input);

    Eigen::Vector2f phiTheta = X_tilda.transpose()*P*b*y;
    float phiSigma = (X_tilda.transpose()*P*b);
    float phiOmega = (X_tilda.transpose()*P*b);
    phiOmega = phiOmega*L1_Input;

    sigmaHat_d = this->projection_operator(sigmaHat,-sigmaGain*phiSigma,sigmaEpsilon,sigmaMax,sigmaMin);
    thetaHat_d(0) = this->projection_operator(thetaHat(0),-thetaGain*phiTheta(0),thetaEpsilon,thetaMax,thetaMin);
    thetaHat_d(1) = this->projection_operator(thetaHat(1),-thetaGain*phiTheta(1),thetaEpsilon,thetaMax,thetaMin);
    omegaHat_d = this->projection_operator(omegaHat,-omegaGain*phiOmega,omegaEpsilon,omegaMax,omegaMin);

    
    sigmaHat = this->trapezoidal_integration(sigmaHat,sigmaHat_d,dt,sigmaHat_d_last);
    thetaHat(0) = this->trapezoidal_integration(thetaHat(0),thetaHat_d(0),dt,thetaHat_d_last(0));
    thetaHat(1) = this->trapezoidal_integration(thetaHat(1),thetaHat_d(1),dt,thetaHat_d_last(1));
    omegaHat = this->trapezoidal_integration(omegaHat,omegaHat_d,dt,omegaHat_d_last);
    
    // Constraining variables to their limits
    sigmaHat = this->constrain_float(sigmaHat,sigmaMax,sigmaMin);
    thetaHat(0) = this->constrain_float(thetaHat(0),thetaMax,thetaMin);
    thetaHat(1) = this->constrain_float(thetaHat(1),thetaMax,thetaMin);
    omegaHat = this->constrain_float(omegaHat,omegaMax,omegaMin);
    
    rg = Kg*refState(2); // Pitch angle

   
    float eita = (omegaHat*L1_Input) + (thetaHat.transpose()*y)+ sigmaHat;
    float filterInput = (eita-rg);

   
    // std::cout << "rg:" << std::endl << rg << std::endl;

    // float eita = this->eitaHatFn(L1_Input, y);

    // doubleDerivativeVector.push_back(eita);
    // derivativeVector.push_back(eita);

    // float eita_dd = doubleDerivativeFilt.filter(doubleDerivativeVector);
    // float eita_d = derivativeFilt.filter(derivativeVector);
    // float eita_filtered = 0.9588*(-eita_dd-(6.5*eita_d)-(10.5*eita));

    // eita_d2 = (eita-eita_last)/dt;
    // eita_last = eita;

    // eita_dd2 = (eita_d2-eita_d_last)/dt;
    // eita_d_last = eita_d2;

    // PlotingVector.data[3] = eita_dd;
    // PlotingVector.data[0] = eita_filtered;
    
    // float filterInput = (rg(0)-eita_filtered);
    L1_Input = -1*bq3.step(filterInput);
    // inputVector.push_back(L1_Input);
    // L1_Input = inputFilt.filter(inputVector);
    // L1_Input = this->constrain_float(L1_Input,30,-30);
    


    // PlotingVector.data[0] = filterInput;
    // PlotingVector.data[2] = L1_Input;

    // trq_r.data = L1_Input;

    // Lwheel_pub.publish(trq_r);
    // Rwheel_pub.publish(trq_r);

    // this->dynamicstest(L1_Input);
    
    return (L1_Input);

 } // End of L1Controller

// float igor_l1_control::eitaHatFn(float u, Eigen::VectorXf y){

//     Xu_dot = (Az*Xu)+(Bz*u);
//     Uz = (Cz*Xu)+(Dz*u);
//     Xg_hat << Xv_hat,Xu;
//     Xg_Norm = Xg_hat.lpNorm<Eigen::Infinity>();
//     e_y = Bhat_tr*Cm_tr*Py*Y_tilda;
//     eitaHat = omegaHat*Uz(0) + thetaHat*Xg_Norm + sigmaHat;
    
//     Xu(0) = this->trapezoidal_integration(Xu(0),Xu_dot(0),dt,Xu_dot_last(0));
//     Xu(1) = this->trapezoidal_integration(Xu(1),Xu_dot(1),dt,Xu_dot_last(1));  
    
    

//     Xv_hat = V_hat + H*y;
//     Y_tilda = y_hat-y;
//     V_hat = this->V_hat_fn(y,Y_tilda);
//     y_hat = this->y_hat_fn(eitaHat,Y_tilda);
    

//     sigmaHat_d = this->projection_operator(sigmaHat,-sigmaGain*e_y(0),sigmaEpsilon,sigmaMax,sigmaMin);
//     thetaHat_d = this->projection_operator(thetaHat,-thetaGain*Xg_Norm*e_y(0),thetaEpsilon,thetaMax,thetaMin);
//     omegaHat_d = this->projection_operator(omegaHat,-omegaGain*Uz(0)*e_y(0),omegaEpsilon,omegaMax,omegaMin);
    

//     sigmaHat = this->trapezoidal_integration(sigmaHat,sigmaHat_d,dt,sigmaHat_d_last);
//     thetaHat = this->trapezoidal_integration(thetaHat,thetaHat_d,dt,thetaHat_d_last);
//     omegaHat = this->trapezoidal_integration(omegaHat,omegaHat_d,dt,omegaHat_d_last);
    
//     // Constraining variables to their limits
//     sigmaHat = this->constrain_float(sigmaHat,sigmaMax,sigmaMin);
//     thetaHat = this->constrain_float(thetaHat,thetaMax,thetaMin);
//     omegaHat = this->constrain_float(omegaHat,omegaMax,omegaMin);



//     return eitaHat;

// } // End of eitaHat

// Eigen::VectorXf igor_l1_control::V_hat_fn(Eigen::Vector2f y, Eigen::Vector2f y_tilde){

//     V_hat_d = Av*Xv_hat-(Kv*y)-(PvInv*Am_tr*Cm_tr*Py*y_tilde);
//     V_hat(0) = this->trapezoidal_integration(V_hat(0),V_hat_d(0),dt,V_hat_d_last(0));
//     V_hat(1) = this->trapezoidal_integration(V_hat(1),V_hat_d(1),dt,V_hat_d_last(1));
//     V_hat(2) = this->trapezoidal_integration(V_hat(2),V_hat_d(2),dt,V_hat_d_last(2));
//     V_hat(3) = this->trapezoidal_integration(V_hat(3),V_hat_d(3),dt,V_hat_d_last(3));

//     // PlotingVector.data[0] = Xv_hat(2);
//     // PlotingVector.data[1] = Xv_hat(3);

//     return V_hat;

// }

// Eigen::VectorXf igor_l1_control::y_hat_fn(float eitaHat, Eigen::Vector2f y_tilde){

//     y_hat_dot = (-alpha*y_tilde)+(Cm*Am*Xv_hat)+(Cm*Bhat*eitaHat);
//     y_hat(0) = this->trapezoidal_integration(y_hat(0),y_hat_dot(0),dt,y_hat_dot_last(0));
//     y_hat(1) = this->trapezoidal_integration(y_hat(1),y_hat_dot(1),dt,y_hat_dot_last(1));

//     // PlotingVector.data[0] = y_hat(0);
//     // PlotingVector.data[1] = y_hat(1);

//     return y_hat;

// }


Eigen::VectorXf igor_l1_control::lqr_controller (Eigen::VectorXf vec) //LQR State-feedback controller
{
    ROS_INFO("In LQR");
    //ROS_INFO("Pitch angle: %f", igor_state(2));

    if (igorState(2)>= -0.35 && igorState(2) <= 0.35){
        
        //igor_knee_control::ref_update();

        Trq_lqr(0) =  (k_r*(refState-vec)).value(); // taking the scalar value of the eigen-matrx
      
        Trq_lqr(1) =  (k_l*(refState-vec)).value();
        

        // Lwheel_pub.publish(trq_l); // Publish left wheel torque
        // Rwheel_pub.publish(trq_r); // Publish right wheel torque

    

       
    }
    else if (igorState(2)<= -1.4 || igorState(2) >= 1.4){
        Trq_lqr(0) = 0;
        Trq_lqr(1) = 0;
        // Lwheel_pub.publish(trq_l);
        // Rwheel_pub.publish(trq_r);
    }

    // PlotingVector.data[0] = (k_r*(refState-vec)).value();

    return Trq_lqr;

    
} // End of lqr_controller

void igor_l1_control::augmented_controller(Eigen::VectorXf eig_vec){

    y(0) = eig_vec(2); // Pitch angle
    y(1) = eig_vec(5); // Pitch rate
    Eigen::Vector2f Trq_lqr = this->lqr_controller(eig_vec); // Lqr control
    float Ua = this->L1ControlInput(y); // Adaptive control

    trq_r.data = Trq_lqr(0)+Ua;
    trq_l.data = Trq_lqr(1)+Ua;

    Rwheel_pub.publish(trq_r);
    Lwheel_pub.publish(trq_l);

    PlotingVector.data[0] = Trq_lqr(0);
    PlotingVector.data[1] = Trq_lqr(1);
    PlotingVector.data[2] = Ua;
    


}// End of augmented_controller

float igor_l1_control::trapezoidal_integration(float yLast, float y_dot, float dt, float &y_dot_last){

    float y1 = yLast + (dt/2)*(y_dot_last+y_dot);
    y_dot_last = y_dot;

    return y1;

} // End trapezoidal_integration

// void igor_l1_control::dynamicstest(float u){
//     X_dot_test = A*X_test + B*u;
//     X_test(0) = this->trapezoidal_integration(X_test(0),X_dot_test(0),dt,X_dot_test_last(0));
//     X_test(1) = this->trapezoidal_integration(V_hat(1),X_dot_test(1),dt,X_dot_test_last(1));
//     X_test(2) = this->trapezoidal_integration(V_hat(2),X_dot_test(2),dt,X_dot_test_last(2));
//     X_test(3) = this->trapezoidal_integration(V_hat(3),X_dot_test(3),dt,X_dot_test_last(3));

//     // PlotingVector.data[0] =  X_test(0);
//     // PlotingVector.data[1] =  X_test(1);

//     // std::cout << "X_dot_test:" << std::endl << X_dot_test << std::endl;
//     // std::cout << "X_test:" << std::endl << X_test << std::endl;
    
// }


// float igor_l1_control::Proj2(float theta_, float y_, float Pm_, float PBar_, float epsilon_){

//     float Psi = (2/epsilon_) * (pow((theta_-PBar_)/Pm_,2)-1+epsilon_);
//     float PsiGradient = (4/epsilon_)*((theta_-PBar_)/Pm_);
//     float projection = 0;
//     float AbsPsiGrad = abs(PsiGradient);

//     if(Psi <= 0){

//         projection = y_;

//     }
//     else if (Psi >= 0 && (PsiGradient*y_)<=0 ){

//         projection = y_;

//     }

//     else{

//         projection = y_-((Psi*PsiGradient*y_*PsiGradient)/pow(AbsPsiGrad,2));
//     }

//     return projection;
// }

float igor_l1_control::projection_operator(float Theta, float phi, float epsilon, float th_max, float th_min) const
{

    // Calculate convex function
    // Nominal un-saturated value is above zero line on a parabolic curve
    // Steepness of curve is set by epsilon
    float f_diff2 = (th_max-th_min)*(th_max-th_min);
    float f_theta = (-4*(th_min - Theta) * (th_max - Theta))/(epsilon*f_diff2);
    float f_theta_dot = (4*(th_min + th_max - (2*Theta)))/(epsilon*f_diff2);

    float projection_out = phi;

    if (f_theta <= 0 && f_theta_dot*phi < 0)
    {
        projection_out = phi*(f_theta+1); 
    }

    return projection_out;
} // End of Projection operator

float igor_l1_control::constrain_float(float var, float max, float min){

    if (var>max)
    {
        return max;
    }

    else if (var<min)
    {
        return min;
    }

    return var;
} // End of constrain_float





igor_l1_control::~igor_l1_control()
{
    std::cout<<"igor_l1_control object destroyed" << std::endl;
} // End of destructor

int main(int argc, char **argv){ /** The arguments to main are the means by which a system outside scope 
and understanding of your program can configure your program to do a particular task. These are command line parameters.**/


ros::init(argc, argv, "igor_l1_controller"); // node name, can be superseded by node name in the launch file
ros::NodeHandle nh;
igor_l1_control myNode(&nh); // creating the igor_l1_control object

ros::Duration(0.1).sleep();
//ros::spin(); // only need for subscriber to call its callback function (Does not need for Publisher).
ros::MultiThreadedSpinner spinner(3); // Use 6 threads for 6 callbacks in parallel
spinner.spin(); // spin() will not return until the node has been shutdown

return 0;

} // end of main