cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# remove this is using an Arduino without native USB (e.g. other than Leonardo)
#add_definitions(-DUSB_CON)

print_board_list()

generate_arduino_firmware(phantomx_turret
  SRCS phantomx_turret.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD ArbotiX\ Std
  PORT /dev/ttyUSB0
)
