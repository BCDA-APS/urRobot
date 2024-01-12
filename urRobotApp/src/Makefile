TOP=../..

include $(TOP)/configure/CONFIG

# include ur_rtde library headers
INC += ur_rtde/dashboard_client.h
INC += ur_rtde/dashboard_enums.h
INC += ur_rtde/robot_state.h
INC += ur_rtde/robotiq_gripper.h
INC += ur_rtde/robotiq_gripper.h
INC += ur_rtde/rtde.h
INC += ur_rtde/rtde_control_interface.h
INC += ur_rtde/rtde_control_interface_doc.h
INC += ur_rtde/rtde_control_script.h
INC += ur_rtde/rtde_export.h
INC += ur_rtde/rtde_io_interface.h
INC += ur_rtde/rtde_io_interface_doc.h
INC += ur_rtde/rtde_receive_interface.h
INC += ur_rtde/rtde_receive_interface_doc.h
INC += ur_rtde/rtde_utility.h
INC += ur_rtde/script_client.h
INC += urcl/tcp_socket.h
INC += urcl/tcp_server.h
INC += urcl/script_sender.h

# paths relative to where they get installed (./O.rhel9-x86_64)
LIB_INSTALLS += ../os/rhel9/librtde.a
BIN_INSTALLS += ../os/rhel9/librtde.so
BIN_INSTALLS += ../os/rhel9/librtde.so.1.5
BIN_INSTALLS += ../os/rhel9/librtde.so.1.5.7

LIBRARY_IOC += urRobot

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install urRobotSupport.dbd into <top>/dbd
DBD += urRobotSupport.dbd

urRobotSupport_DBD += urRobot.dbd

# specify all source files to be compiled and added to the library
urRobot_SRCS += rtde_driver.cpp dashboard_driver.cpp

urRobot_LIBS += asyn
urRobot_LIBS += rtde
urRobot_LIBS += $(EPICS_BASE_IOC_LIBS)

urRobot_SYS_LIBS += boost_system
urRobot_SYS_LIBS += boost_thread

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
