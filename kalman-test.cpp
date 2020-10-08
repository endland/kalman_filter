/**
 * Test for the KalmanFilter class with 1D projectile motion.
 *
 * @author: Hayk Martirosyan
 * @date: 2014.11.15
 */

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <random>
#include <chrono>

#include "kalman.hpp"
#include "pbPlots.hpp"
#include "supportLib.hpp"

using namespace std;
using namespace Eigen;

#define WIDTH 800
#define HEIGHT 600

//std::default_random_engine generator; 
//std::random_device rd;
//std::mt19937_64 rng(rd());

/*
float randn_single(float _mean, float _var)
{
  float rand_n;

  //std::default_random_engine generator;
  std::normal_distribution<float> distribution(_mean, _var);

  rand_n = distribution(generator);
  //std::cout<< "generated randn value : " << rand_n << std::endl;
  //std::cout<< "unifor_random value : " << uniform_random() << std::endl;

  return rand_n;
}
*/

vector<double> randn(float _mean, float _var, int size)
{
  vector<double> rand_n;
  std::random_device rd;
  std::mt19937_64 rng(rd());

  std::normal_distribution<float> distribution(_mean, _var);
  //std::cout<< "generated randn value : " << std::endl;
  for (int i = 0; i < size; i++){
    //rand_n.push_back(distribution(generator));
    rand_n.push_back(distribution(rng));
    //std::cout<< " "<< rand_n[i];
  }
  std::cout<<std::endl;
  
  //std::cout<< "generated randn value : " << rand_n << std::endl;
  //std::cout<< "unifor_random value : " << uniform_random() << std::endl;

  return rand_n;
}

int main(int argc, char* argv[]) {

  vector<double> rads;
  vector<double> Sdata_ans;        //True data
  vector<double> Sdata;            //True data + noise
  vector<double> Sdata_kf;         //kalman filtered data
  vector<double> noise;

  ScatterPlotSeries *sp_true = GetDefaultScatterPlotSeriesSettings();

  sp_true->xs = &rads;
  sp_true->ys = &Sdata_ans;
  sp_true->linearInterpolation = true;
  sp_true->lineType = toVector(L"solid");
  //true_data->pointType = toVector(L"triangles");
  sp_true->lineThickness = 1;
  sp_true->color = GetBlack();

  ScatterPlotSettings *true_setting = GetDefaultScatterPlotSettings();
  true_setting->width = WIDTH;
  true_setting->height = HEIGHT;
  true_setting->autoBoundaries = true;
  true_setting->autoPadding = true;
  true_setting->title = toVector(L"Sensor_Data");
  true_setting->xLabel = toVector(L"Amplitude");
  true_setting->yLabel = toVector(L"Time");
  true_setting->scatterPlotSeries->push_back(sp_true);

  
  ScatterPlotSeries *sp_noise = GetDefaultScatterPlotSeriesSettings();

  sp_noise->xs = &rads;
  sp_noise->ys = &Sdata;
  sp_noise->linearInterpolation = false;
  sp_noise->lineType = toVector(L"solid");
  sp_noise->pointType = toVector(L"triangles");
  sp_noise->lineThickness = 1;
  sp_noise->color = GetGray(0.3);

  true_setting->scatterPlotSeries->push_back(sp_noise);


  ScatterPlotSeries *sp_kf = GetDefaultScatterPlotSeriesSettings();

  sp_kf->xs = &rads;
  sp_kf->ys = &Sdata_kf;
  sp_kf->linearInterpolation = false;
  sp_kf->lineType = toVector(L"solid");
  sp_kf->pointType = toVector(L"circles");
  sp_kf->lineThickness = 1;
  //sp_kf->color = GetGray(0.6);
  sp_kf->color = CreateRGBColor(0.1, 0.1, 1);

  true_setting->scatterPlotSeries->push_back(sp_kf);

  noise = randn(0, 0.3, 1001);
  //std::cout<< "size of noise : " << noise.size() << std::endl;
 

  for (int i=0; i < 1001; i++) 
  {
     rads.push_back(M_PI/180*i);
     //Sdata_ans.push_back(sin(2.f*M_PI*rads[i]));
     Sdata_ans.push_back(sin(rads[i]));

     //Sdata.push_back((Sdata_ans[i])+randn(0, 0.3));
     Sdata.push_back((Sdata_ans[i])+noise[i]);

  }



  //int n = 3; // Number of states
  int n = 2; // Number of states
  int m = 1; // Number of measurements

  //double dt = 1.0/50; // Time step
  double dt = M_PI/180; // Time step

  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd C(m, n); // Output matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  // Discrete LTI projectile motion, measuring position only
  //A << 1, dt, 0, 0, 1, dt, 0, 0, 1;               // F
  A << 1, dt, 0, 1;               // F
  //C << 1, 0, 0;                                   // H
  C << 1, 0;                                   // H

  // Reasonable covariance matrices
  //Q << .05, .05, .05, .05;
  Q << pow(.01, 2), pow(.01, 2), pow(.01, 2), pow(.01, 2);
  //R << 0.5;
  R << pow(0.1, 2);
  //P << .1, .1, .1, .1;
  P << 0., 1., 0., 0.;

  std::cout << "A: \n" << A << std::endl;
  std::cout << "C: \n" << C << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "R: \n" << R << std::endl;
  std::cout << "P: \n" << P << std::endl;

  // Construct the filter
  KalmanFilter kf(dt, A, C, Q, R, P);
  
  Eigen::VectorXd x0(n);
  
  x0 << Sdata[0], 0;
  cout<<"x0 = "<<x0<<endl;
 
  kf.init(0, x0);

  // Feed measurements into filter, output estimated states
  double t = 0;
  Eigen::VectorXd y(m);
  //cout<<"sin(30deg)"<<sin(0.52359877)<<endl;
  //std::cout << "t = " << t << ", " << "x_hat[0]: " << kf.state().transpose() << std::endl;
  //for(int i = 0; i < measurements.size(); i++) {
  for(int i = 0; i < Sdata.size(); i++) {
    t += dt;
    //y << measurements[i];
    y << Sdata[i];
    kf.update(y);
    //std::cout << "t = " << t << ", " << "y[" << i << "] = " << y.transpose()<< ", x_hat[" << i << "] = " << kf.state().transpose()[0] << std::endl;
    Sdata_kf.push_back(kf.state().transpose()[0]);
  }

  RGBABitmapImageReference *imageRef = CreateRGBABitmapImageReference();
  RGBABitmapImage *combined = CreateImage(WIDTH, HEIGHT, GetWhite());
  RGBABitmapImage *ans, *sensor;

  DrawScatterPlotFromSettings(imageRef, true_setting);
  ans = imageRef->image;

  DrawImageOnImage(combined, ans, 0, 0);

  vector<double> *pngData = ConvertToPNG(combined);
  WriteToFile(pngData, "plot_cpp.png");
  DeleteImage(imageRef->image);

  return 0;
}
