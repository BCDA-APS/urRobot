TOP=../..

include $(TOP)/configure/CONFIG

LIBRARY_IOC += urRobot

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install cpsc.dbd into <top>/dbd
DBD += urRobotSupport.dbd

urRobotSupport_DBD += urRobot.dbd

# specify all source files to be compiled and added to the library
urRobot_SRCS += ur_driver.cpp

urRobot_LIBS += asyn
urRobot_LIBS += $(EPICS_BASE_IOC_LIBS)


include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
