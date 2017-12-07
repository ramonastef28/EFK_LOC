#ifndef KALMAN_H_
#define KALMAN_H_
#include "Eigen/Dense"
#include "helper_functions.h"

class KalmanFilter{

public:
   //state vector
   Eigen::VectorXd mu_;
   //state covariance matrix
   Eigen::MatrixXd P_;
   //state transition matrix
   Eigen::MatrixXd F_;
   //process covariance matrix
   Eigen::MatrixXd Q_;
   //measurement matrix
   Eigen::MatrixXd H_;
   //measeurement covariance matrix
   Eigen::MatrixXd R_;
   //GPS coord
   Eigen::VectorXd mx_;
   Eigen::VectorXd my_;
   Eigen::VectorXd diffds_;   
   
   const float dt_ = 1.0/100.0;  
   const float dt_vel_ = 1.0/30.0;
   const float dt_gps = 1.0/30.0;

  /**
    * Constructor
    */
   KalmanFilter();

   /**
    * Destructor
    */
   virtual ~KalmanFilter();	
   void Init(Eigen::VectorXd mu_init, Eigen::MatrixXd P_init,Eigen::MatrixXd Q_init); 
   void TransfGPS(std::vector<GPS> gps_data);
   void Predict();
   void Update(measurement z, int index);
   //void Predict(const double &delta_rot1, const double &delta_trans, const double &delta_rot2);

   //void Update(std::vector<Landmarks> world, std::vector<SLandmarks> z);
};
#endif 
