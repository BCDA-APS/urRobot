# NOTE: paths relative to where they get installed (e.g. ./O.rhel9-x86_64)

TOP=../..

include $(TOP)/configure/CONFIG

# include ur_rtde library headers
USR_INCLUDES += -I../ur_rtde
USR_INCLUDES += -I../urcl
USR_INCLUDES += -I../spdlog

LIB_INSTALLS += ../os/rhel9/librtde.a
BIN_INSTALLS += ../os/rhel9/librtde.so
BIN_INSTALLS += ../os/rhel9/librtde.so.1.5
BIN_INSTALLS += ../os/rhel9/librtde.so.1.5.7

# Define IOC library name
LIBRARY_IOC += urRobot

# install urRobotSupport.dbd into <top>/dbd
DBD += urRobotSupport.dbd
urRobotSupport_DBD += urRobot.dbd

# source files to be compiled and added to the library
urRobot_SRCS += dashboard_driver.cpp
urRobot_SRCS += rtde_receive_driver.cpp
urRobot_SRCS += rtde_io_driver.cpp
urRobot_SRCS += rtde_control_driver.cpp
urRobot_SRCS += gripper_driver.cpp

# Libraries needed for urRobot
urRobot_LIBS += asyn
urRobot_LIBS += rtde
urRobot_LIBS += $(EPICS_BASE_IOC_LIBS)
urRobot_SYS_LIBS += boost_system
urRobot_SYS_LIBS += boost_thread

# Tool for calibrating the Robotiq Gripper
PROD += calibrate_gripper
calibrate_gripper_SRCS += calibrate_gripper.cpp
calibrate_gripper_LIBS += rtde

# Tool for tweaking the robot's pose with your keyboard keys
PROD += keyboard_tweak
keyboard_tweak_SRCS += keyboard_tweak.cpp
keyboard_tweak_LIBS += pvAccessCA ca pvAccess pvData Com
keyboard_tweak_SYS_LIBS += ncurses

# Program for testing ur_rtde by itself without asyn
TESTPROD += urtest
urtest_SRCS += urtest.cpp
urtest_LIBS += rtde
urtest_SYS_LIBS += boost_system
urtest_SYS_LIBS += boost_thread

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
