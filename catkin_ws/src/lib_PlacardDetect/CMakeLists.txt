#--------------------------------------------------------
# The CMakeLists.txt for:               lib_PlacardDetect
# Author(s):                                   Alon Yaari
#--------------------------------------------------------


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


FILE(GLOB SRC *.cpp)

# Build Library
ADD_LIBRARY(PlacardDetect ${SRC})

# Install Headers
#INSTALL( FILES ${HEADERS} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/lib_PlacardDetect)

# Install Libararies
INSTALL( TARGETS PlacardDetect
   RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
   LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
   ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)
