TOP=../

include $(TOP)/configure/CONFIG

SRC_DIRS += $(TOP)/urRobotApp/src

TESTPROD_HOST += testServoTrajectory
testServoTrajectory_SRCS += test_servo_trajectory.cpp
testServoTrajectory_SRCS += servo_trajectory.cpp
testServoTrajectory_LIBS += Com
TESTS += testServoTrajectory

TESTSCRIPTS_HOST += $(TESTS:%=%.t)

include $(TOP)/configure/RULES
