CMAKE_MINIMUM_REQUIRED(VERSION 3.12)

FIND_PACKAGE(Eigen3 3.3 REQUIRED NO_MODULE)

ADD_SUBDIRECTORY(lib)
ADD_SUBDIRECTORY(test)