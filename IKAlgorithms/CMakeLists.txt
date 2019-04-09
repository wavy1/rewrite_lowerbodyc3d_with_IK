PROJECT(IKAlgorithms)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.2)

SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_MODULE_PATH ${IKAlgorithms_SRCS})
SET(BTK_DIR ~/BTKCore/build)
SET(EIGEN2_DIR ~/eigen-git-mirror)

FIND_PACKAGE(BTK REQUIRED)
INCLUDE(${BTK_USE_FILE})
INCLUDE_DIRECTORIES(${EIGEN2_DIR}/Eigen/src)

SET(IKAlgorithms_SRCS main.cpp FabrikSolve.cpp FabrikSolve.h AcquisitionChain.cpp AcquisitionChain.h Joint3D.cpp Joint3D.h)
ADD_EXECUTABLE(IKAlgorithms ${IKAlgorithms_SRCS})
TARGET_LINK_LIBRARIES(IKAlgorithms BTKIO)
