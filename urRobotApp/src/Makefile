TOP=../..

include $(TOP)/configure/CONFIG

# include ur_rtde library headers
USR_INCLUDES += -Iur_rtde
USR_INCLUDES += -Iurcl

# paths relative to where they get installed (./O.rhel9-x86_64)
LIB_INSTALLS += ../os/rhel9/librtde.a
BIN_INSTALLS += ../os/rhel9/librtde.so
BIN_INSTALLS += ../os/rhel9/librtde.so.1.5
BIN_INSTALLS += ../os/rhel9/librtde.so.1.5.7

LIBRARY_IOC += urRobot

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
