/**
 * Simple Linear Kalman-filter example
 *
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
  vector<double> Sdata;            //True data + noise (measurement data)
  vector<double> Sdata_kf;         //kalman filtered data
  vector<double> noise;            //Gaussian noise

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
  true_setting->xLabel = toVector(L"Position");
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
  int n = 2; // Number of states      x = [r, v]
  int m = 1; // Number of measurements

  //double dt = 1.0/50; // Time step
  double dt = M_PI/180; // Time step

  Eigen::MatrixXd A(n, n); // System dynamics matrix,       2*2
  Eigen::MatrixXd H(m, n); // Measurement matrix,           1*2
  Eigen::MatrixXd Q(n, n); // Process noise covariance,     2*2
  Eigen::MatrixXd R(m, m); // Measurement noise covariance, 1*1
  Eigen::MatrixXd P(n, n); // Estimate error covariance,    2*2

  A << 1, dt, 0, 1;               // A = [1, dt
                                  //      0, 1]
  
  H << 1, 0;                                   // H = [1, 0]

  // Reasonable covariance matrices
  Q << pow(.01, 2), pow(.01, 2), pow(.01, 2), pow(.01, 2);
  
  R << pow(0.1, 2);
  //P << .1, .1, .1, .1;
  P << 0., 1., 0., 0.;                // P = [0, 1
                                      //      0, 0]

  std::cout << "A: \n" << A << std::endl;
  std::cout << "H: \n" << H << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "R: \n" << R << std::endl;
  std::cout << "P: \n" << P << std::endl;

  // Construct the filter
  KalmanFilter kf(dt, A, H, Q, R, P);
  
  Eigen::VectorXd x0(n);           // x = [r, v], 1*2
  
  x0 << Sdata[0], 0;               // set initial state with first measurement data
  cout<<"x0 = "<<x0<<endl;
 
  kf.init(0, x0);

  // Feed measurements into filter, output estimated states
  double t = 0;
  Eigen::VectorXd y(m);

  for(int i = 0; i < Sdata.size(); i++) {
    t += dt;
    y << Sdata[i];
    kf.update(y);
    Sdata_kf.push_back(kf.state().transpose()[0]);
  }

  RGBABitmapImageReference *imageRef = CreateRGBABitmapImageReference();
  RGBABitmapImage *combined = CreateImage(WIDTH, HEIGHT, GetWhite());
  RGBABitmapImage *result;

  DrawScatterPlotFromSettings(imageRef, true_setting);
  result = imageRef->image;

  DrawImageOnImage(combined, result, 0, 0);

  vector<double> *pngData = ConvertToPNG(combined);
  WriteToFile(pngData, "plot_true_measurement_kf.png");
  DeleteImage(imageRef->image);

  return 0;
}
