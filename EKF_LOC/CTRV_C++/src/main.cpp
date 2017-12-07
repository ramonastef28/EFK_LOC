#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include <math.h>
#include "Eigen/Dense"
#include "kalman_ctrv.h"
#include "helper_functions.h"
//#include "utm.h"
#include "LatLong-UTMconversion.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

	
void check_arguments(int argc, char* argv[]){
  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/GPS vel yaw_rate output.txt";

  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (argc == 1) {
    cerr << usage_instructions << endl;
  } else if (argc == 2) {
    cerr << "Please include the input files.\n" << usage_instructions << endl;
  } else if (argc == 3) {
    cerr << "Please include the input files.\n" << usage_instructions << endl;
  } else if (argc == 4) {
    cerr << "Please include the input files.\n" << usage_instructions << endl;
  } else if (argc == 5) {
    has_valid_args = true;
  } else if (argc > 3) {
    cerr << "Too many arguments.\n" << usage_instructions << endl;
  }
 
  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}

void check_files(ifstream& data_file, string& data_name, ofstream& out_file, string& out_name){

  if (!data_file.is_open()) {
    cerr << "Cannot open input file: " << data_name << endl;
    exit(EXIT_FAILURE);
  }
  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }

}


int main(int argc, char* argv[]) {
  KalmanFilter Kalman;
  float dt = Kalman.dt_; 
  check_arguments(argc, argv);
  
  string gps_name_ = argv[1];
  ifstream in_file_gps_(gps_name_.c_str(), ifstream::in);

  string vel_name_ = argv[2];
  ifstream in_file_vel_(vel_name_.c_str(), ifstream::in);

  string yaw_rate_name_ = argv[3];
  ifstream in_file_yaw_rate_(yaw_rate_name_.c_str(), ifstream::in);

  string out_file_name_ = argv[4];
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);
 
  check_files(in_file_gps_, gps_name_, out_file_, out_file_name_);

  string line;
  VectorXd mu_init = VectorXd(5);
  MatrixXd P_init = MatrixXd::Identity(5,5);
   // motion noise
  MatrixXd Q_init = MatrixXd(5,5);
  P_init << 1000.0, 0.0, 0.0, 0.0, 0.0,
	    0.0, 1000.0, 0.0, 0.0, 0.0,
	    0.0, 0.0, 1000.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1000.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1000.0;
  float sGPS = 0.5*8.8*dt*dt; // assume 8.8m/s2 as maximum acceleration
  float sCourse = 0.1*dt;
  float sVelocity = 8.8*dt;
  float sYaw = 1.0*dt; 	  
  Q_init << sGPS*sGPS, 0.0, 0.0, 0.0, 0.0,
         0.0, sGPS*sGPS, 0.0, 0.0, 0.0,
         0.0, 0.0, sCourse, 0.0, 0.0,
	 0.0, 0.0, 0.0, sVelocity, 0.0,
         0.0, 0.0, 0.0, 0.0, sYaw;
  vector<GPS> gps_data; 
  vector<measurement> z;
  int RefEllipsoid = 23;//WGS-84. See list with file "LatLong- UTM conversion.cpp" for id numbers
  char UTMZone[4];  
  cout << setiosflags(ios::showpoint | ios::fixed) << setprecision(12);
  while (getline(in_file_gps_,line)) {
     istringstream iss(line);
     GPS gps_line;
     double lat, lon, alt, course, UTMNorthing, UTMEasting;
     iss >> lat;
     iss >> lon;
     iss >> alt;
     iss >> course;
     LLtoUTM(RefEllipsoid, lat, lon, UTMNorthing, UTMEasting, UTMZone);
     gps_line.lat = lat;
     gps_line.lon = lon;
     gps_line.y = UTMNorthing;
     gps_line.x = UTMEasting;
     gps_line.h = alt;	
     gps_line.course = -course*10e-5 + 90.0;
     gps_data.push_back(gps_line);		
  } 

  vector<double> velocity_data;
  while (getline(in_file_vel_,line)) {
     istringstream iss(line);
     double velocity;  
     iss >> velocity;
     velocity_data.push_back(velocity);
  } 

  vector<double> yaw_rate; 
  while (getline(in_file_yaw_rate_,line)) {
     istringstream iss(line);
     double yaw_vel;
     iss >> yaw_vel;
     yaw_rate.push_back(yaw_vel);
  } 
  cout << setiosflags(ios::showpoint | ios::fixed) << setprecision(8);
  

 
  measurement meas_line; 	
  cout << "size of gps data: " << gps_data.size() << endl;
  int k_gps=0;
  int k_vel=0;
 
  Kalman.TransfGPS(gps_data);
 
  mu_init << gps_data[0].x, gps_data[0].y, M_PI-gps_data[0].course/180.0*M_PI, velocity_data[0], yaw_rate[0];
  cout << "yaw course initial: " << mu_init[4] << endl; 
  Kalman.Init(mu_init, P_init, Q_init);
  Kalman.Predict();
  out_file_ << std::setprecision(std::numeric_limits<double>::digits10 + 2) << Kalman.mu_[0] << "\t";
  out_file_ << std::setprecision(std::numeric_limits<double>::digits10 + 2) << Kalman.mu_[1] << "\t";
  out_file_ << Kalman.mu_[2] << "\t";
  out_file_ << Kalman.mu_[3] << "\t";
  out_file_ << Kalman.mu_[4] << "\n";
 
 
  for (int i=1; i<yaw_rate.size(); ++i){
     if (i % 3 == 0) {
       //cout << "k_vel: " << k_vel << endl; 
       k_vel = k_vel + 1;
       Kalman.mu_[3] = velocity_data[k_vel];
     }
     Kalman.mu_[3] = velocity_data[k_vel];
     Kalman.mu_[4] = yaw_rate[i];
     Kalman.Predict();
     out_file_ << std::setprecision(std::numeric_limits<double>::digits10 + 2) << Kalman.mu_[0] << "\t";
     out_file_ << std::setprecision(std::numeric_limits<double>::digits10 + 2) << Kalman.mu_[1] << "\t";
     out_file_ << Kalman.mu_[2] << "\t";
     out_file_ << Kalman.mu_[3] << "\t";
     out_file_ << Kalman.mu_[4] << "\n";
  } 

 
// close files
  if (out_file_.is_open()) {
    out_file_.close();
  }

  if (in_file_gps_.is_open()) {
    in_file_gps_.close();
  }
 return 0;

}


