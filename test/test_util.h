#pragma once

#define SMALL_TEST for (unsigned int randomIter = 0; randomIter < 10; randomIter++)
#define MEDIUM_TEST for (unsigned int randomIter = 0; randomIter < 100; randomIter++)
#define BIG_TEST for (unsigned int randomIter = 0; randomIter < 1000; randomIter++)


#include <cstdlib>
#include <ctime>
#include <Eigen/Dense>

inline void seed_random() {
  srand(static_cast <unsigned> (time(0)));
}

inline double random_double(double min = -10.0, double max = 10.0) {
  return min + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(max-min)));
}

template<unsigned int D>
inline Eigen::Matrix<double, D, 1> random_vector(double min = -10.0, double max = 10.0);

template<>
inline Eigen::Matrix<double, 1, 1> random_vector<1>(double min, double max) {
  return Eigen::Matrix<double, 1, 1>( random_double(min, max) );
}

template<>
inline Eigen::Vector2d random_vector<2>(double min, double max) {
  return Eigen::Vector2d( random_double(min, max), random_double(min, max) );
}

template<>
inline Eigen::Vector3d random_vector<3>(double min, double max) {
  return Eigen::Vector3d( random_double(min, max), random_double(min, max), random_double(min, max) );
}

template<unsigned int D>
inline Eigen::Matrix<double, D, D> random_matrix(double min = -10.0, double max = 10.0);

template<>
inline Eigen::Matrix2d random_matrix<2>(double min, double max) {
  Eigen::Matrix2d result;
  result <<
    random_double(min, max), random_double(min, max),
    random_double(min, max), random_double(min, max);
  return result;
}

template<>
inline Eigen::Matrix3d random_matrix<3>(double min, double max) {
  Eigen::Matrix3d result;
  result <<
    random_double(min, max), random_double(min, max), random_double(min, max),
    random_double(min, max), random_double(min, max), random_double(min, max),
    random_double(min, max), random_double(min, max), random_double(min, max);
  return result;
}
