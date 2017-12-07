#include "kalman_ctrv.h"
#include <iostream>
#include "helper_functions.h"
#include <numeric>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd mu_init, MatrixXd P_init, MatrixXd Q_init) {
   mu_ = mu_init;
   P_ = P_init;
   // motion noise
   Q_ = Q_init;	
}

/*
void KalmanFilter::TransfGPS(vector<GPS> gps_data){
  double RadiusEarth = 6378388.0;
  double arc = 2.0*M_PI*(RadiusEarth + gps_data[0].alt)/360.0;
  VectorXd dx, dy, diff_lon, diff_lat;
  int N = gps_data.size(); 
  diff_lon = VectorXd(N-1);
  diff_lat = VectorXd(N-1);
  dx = VectorXd(N);
  dy = VectorXd(N);
  VectorXd ds = VectorXd(N);
  diffds_ = VectorXd::Zero(N);
  mx_ =  VectorXd(N);  
  my_ =  VectorXd(N); 
  dx[0] = 0.0;
  dy[0] = 0.0;
  mx_[0] = 0.0;
  my_[0] = 0.0;
  ds[0] = 0.0; 
  //diffds_[0] = 10; //greater than zero
  for (int i=0; i<N-1; ++i){
        diff_lon[i]= gps_data[i+1].lon - gps_data[i].lon;
        diff_lat[i]= gps_data[i+1].lat - gps_data[i].lat;
	dx[i+1] = arc*cos(gps_data[i].lat*M_PI/180.0)*diff_lon[i];
        dy[i+1] = arc*diff_lat[i];
        mx_[i+1] = mx_[i] + dx[i+1];
	my_[i+1] = my_[i] + dy[i+1];
        ds[i+1] = sqrt(dx[i]*dx[i] + dy[i]*dy[i]);
	diffds_[i] = ds[i+1] - ds[i];
   }
 diffds_[0] = 1;
}
*/

void KalmanFilter::TransfGPS(vector<GPS> gps_data){
  VectorXd dx, dy, diff_x, diff_y;
  int N = gps_data.size();
  diff_x = VectorXd(N-1);
  diff_y = VectorXd(N-1);
  dx = VectorXd(N);
  dy = VectorXd(N);
  VectorXd ds = VectorXd(N);
  diffds_ = VectorXd::Zero(N);
  mx_ =  VectorXd(N);  
  my_ =  VectorXd(N); 
  dx[0] = 0.0;
  dy[0] = 0.0;
  mx_[0] = 0.0;
  my_[0] = 0.0;
  ds[0] = 0.0; 

  for (int i=0; i<N-1; ++i) {
        dx[i+1] = gps_data[i+1].x - gps_data[i].x;
        dy[i+1] = gps_data[i+1].y - gps_data[i].y;
        mx_[i+1] = mx_[i] + dx[i+1];
        my_[i+1] = my_[i] + dy[i+1];
        ds[i+1] = sqrt(dx[i]*dx[i] + dy[i]*dy[i]);
        diffds_[i] = ds[i+1] - ds[i];
        //cout << "diffds : " << diffds_ << endl;
  } 
  diffds_[0] = 1;
}

void KalmanFilter::Predict() {
  float px = mu_[0];
  float py = mu_[1];
  float yaw = mu_[2];
  float v = mu_[3];
  float yaw_rate = mu_[4];

  if (fabs(yaw_rate) < 0.0001) {
      mu_[0] = px + v*dt_ * cos(yaw);
      mu_[1] = py + v*dt_ * sin(yaw);	
      mu_[2] = yaw;
      mu_[3] = v;
      mu_[4] = 0.0000001;			
   } else {
      mu_[0] = mu_[0] + (v/yaw_rate) * (sin(yaw_rate*dt_ + yaw) - sin(yaw)); 
      mu_[1] = mu_[1] + (v/yaw_rate) * (-cos(yaw_rate*dt_ + yaw) + cos(yaw));
      //mu_[2] = fmod(fabs(yaw + yaw_rate * dt_ + M_PI) , (2.0*M_PI)) - M_PI;
      mu_[2] = yaw + yaw_rate*dt_;	
      if (mu_[2] > M_PI) {
	mu_[2] = mu_[2] - 2*M_PI;}
      else if (mu_[2] < -M_PI){
	mu_[2] = mu_[2] + 2*M_PI;}
      else {
	mu_[2] = mu_[2];}
      mu_[3] = v;
      mu_[4] = yaw_rate;	 	
   }
   
  float a13 = (v/yaw_rate)*(cos(yaw_rate*dt_ + yaw) - cos(yaw));
  float a14 = (1/yaw_rate)*(sin(yaw_rate*dt_ + yaw) - sin(yaw));
  float a15 = (dt_*v/yaw_rate)*cos(yaw_rate*dt_ + yaw) -(v/(yaw_rate*yaw_rate))*(sin(yaw_rate*dt_ + yaw) -sin(yaw));
  float a23 = (v/yaw_rate)*(sin(yaw_rate*dt_ + yaw) - sin(yaw));
  float a24 = (1/yaw_rate)*(-cos(yaw_rate*dt_ + yaw) + cos(yaw));
  float a25 = (dt_*v/yaw_rate)*sin(yaw_rate*dt_ + yaw) - (v/(yaw_rate*yaw_rate))*(-cos(yaw_rate*dt_ + yaw) + cos(yaw));
  MatrixXd JA = MatrixXd(5,5);
  JA << 1.0, 0.0, a13, a14, a15,
        0.0, 1.0, a23, a24, a25,
        0.0, 0.0, 1.0, 0.0, dt_,
	0.0, 0.0, 0.0, 1.0, 0.0,
	0.0, 0.0, 0.0, 0.0, 1.0;

 P_ = JA*P_*JA.transpose() + Q_; 
 
 //cout << "mu_ predict: " << mu_ << endl;
 //cout << "P_ before: " << P_ << endl;

}

void KalmanFilter::Update(measurement z, int index){

  //VectorXd h=VectorXd(4);
  MatrixXd JH = MatrixXd(4,5);
  //hx << mu_[0], mu_[1], mu_[3], mu_[4];
  R_ = MatrixXd(4,4);
  float varGPS = 6.0;
  float varspeed = 1.0;
  float varyaw = 0.1;
   
  R_ << varGPS*varGPS, 0.0, 0.0, 0.0,
	0.0, varGPS*varGPS, 0.0, 0.0,
	0.0, 0.0, varspeed, 0.0,
	0.0, 0.0, 0.0, varyaw*varyaw;
  
  if (diffds_(index) > 0) {
	JH << 100, 0.0, 0.0, 0.0, 0.0,
	      0.0, 100, 0.0, 0.0, 0.0,
    	      0.0, 0.0, 0.0, 0.1, 0.0,
	      0.0, 0.0, 0.0, 0.0, 0.1;}
   else{	  
 	JH << 0.0, 0.0, 0.0, 0.0, 0.0,
	      0.0, 0.0, 0.0, 0.0, 0.0,
	      0.0, 0.0, 0.0, 1.0, 0.0,
	      0.0, 0.0, 0.0, 0.0, 1.0; 
  }
  MatrixXd S = MatrixXd(4,4);
  S = JH*P_*JH.transpose() + R_;
  MatrixXd K = MatrixXd(4,5);
  K = (P_*JH.transpose()) *S.inverse();
  //cout << "S: " << S << endl;
  cout << "z.x: " << z.x << endl;
  cout << "z.y: " << z.y << endl; 
  cout << "mu: " << mu_ << endl;	
  VectorXd y = VectorXd(4);
  y[0] = z.x - mu_[0];
  y[1] = z.y - mu_[1];
  y[2] = z.v - mu_[3];
  y[3] = z.yaw_rate - mu_[4];
  cout << "y: " << y << endl; 

  mu_ = mu_ + K*y;
  //cout << "mu_ after: " << mu_ << endl;
 
  MatrixXd I = MatrixXd::Identity(5,5);
 // cout << "P_: " << P_ << endl; 
  P_ = (I - (K*JH))*P_;
  //cout << "P_: after " << P_ << endl;
}



/*
void KalmanFilter::Predict(const double &delta_rot1, const double &delta_trans, const double &delta_rot2) {
 
 // mu_ = VectorXd::Zero(3);
// P_ = MatrixXd::Identity(3,3); 
  MatrixXd G_ = MatrixXd(3,3);  
  // motion noise
  Q_ << 0.2, 0.0, 0.0,
	0.0, 0.2, 0.0,
        0.0, 0.0, 0.02;
*/
/*
  //Jacobian with respect to g
  G_ << 1.0, 0.0, -delta_trans*sin(mu_[2] + delta_rot1),
	0.0, 1.0, delta_trans*cos(mu_[2] + delta_rot1),
	0.0, 0.0, 1.0;

  mu_[0] = mu_[0] + delta_trans*cos(mu_[2] + delta_rot1);
  mu_[1] = mu_[1] + delta_trans*sin(mu_[2] + delta_rot1);
  mu_[2] = mu_[2] +  delta_rot1 +  delta_rot2;

  P_ = G_*P_*G_.transpose() + Q_;
  cout << "x predicted: " << mu_[0] << endl;

}
void KalmanFilter::Update(vector<Landmarks> world, vector<SLandmarks> z){
  double x= mu_[0];
  double y= mu_[1];
  double theta= mu_[2];
  
  int N=z.size();
  double range_exp;
  //vector <double> expected_ranges;
  VectorXd expected_ranges=VectorXd(N);	
  int lm_id, lx, ly; 
  MatrixXd H_ = MatrixXd(N,3);
  VectorXd Z_ = VectorXd(N); 
  if (z.size() != 0){ 
     for (int i=0; i<z.size(); ++i){
        lm_id = z[i].id;
	lx = world[lm_id-1].x;
	ly = world[lm_id-1].y;
	//calculate expected range measurement
	range_exp = sqrt((lx - x)*(lx - x) + (ly - y)*(ly - y));
	H_.row(i) << (x-lx)/range_exp, (y-ly)/range_exp, 0;
  	Z_(i) = z[i].range;
	expected_ranges(i) = range_exp;
      }
      MatrixXd R_= MatrixXd::Identity(N,N);
      MatrixXd I_= MatrixXd::Identity(3,3);
      R_ = 0.5*R_;
      MatrixXd S_ = H_*P_*H_.transpose() + R_;
      MatrixXd K_ = P_*H_.transpose()*S_.inverse();
      //new state
      mu_ = mu_ + K_*(Z_ - expected_ranges);
      P_ = (I_ - K_*H_)*P_;
    }
  else if (z.size() == 0){
      cout << "Initialization! " << endl;
  } 		
}
*/
