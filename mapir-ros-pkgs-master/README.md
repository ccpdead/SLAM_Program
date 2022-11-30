 ### 使用的C++库
 1. Boost
 2. Eigen
 3. MRPT(base， obs, maps, slam)

``` 
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
) 
------------------------------------
target_link_libraries(rf2o_laser_odometry_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${EIGEN_LIBRARIES}
   ${MRPT_LIBS}
)

```