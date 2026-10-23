TOP=../

include $(TOP)/configure/CONFIG

SRC_DIRS += $(TOP)/urRobotApp/src

TESTPROD_HOST += testTrajectory
testTrajectory_SRCS += test_trajectory.cpp
testTrajectory_SRCS += trajectory.cpp
testTrajectory_LIBS += Com
TESTS += testTrajectory

TESTSCRIPTS_HOST += $(TESTS:%=%.t)

include $(TOP)/configure/RULES
