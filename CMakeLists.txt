cmake_minimum_required(VERSION 3.0.2)
project(depthai_hand_tracker)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS rospy)

install(PROGRAMS scripts/depthai_hand_tracker_ros.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# add_custom_target( ${PROJECT_NAME}_install_python_package ALL COMMAND python
# -m pip install -r ${PROJECT_SOURCE_DIR}/requirements.txt )

# if(CATKIN_ENABLE_TESTING) find_package(rostest REQUIRED) #catkin_lint: skip
# find_package( catkin COMPONENTS cube_gazebo cube_navigation REQUIRED )
# add_rostest(test/) endif()
