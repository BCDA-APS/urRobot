# ../../bin/${EPICS_HOST_ARCH}/ur621 st.cmd
< envPaths

epicsEnvSet("PREFIX", "ur621:")

dbLoadDatabase("../../dbd/iocur621Linux.dbd")
iocur621Linux_registerRecordDeviceDriver(pdbbase)

# load the urRobot iocsh example
iocshLoad("$(URROBOT)/iocsh/urRobot.iocsh", "PREFIX=$(PREFIX), IP=164.54.104.148")

###############################################################################
iocInit
###############################################################################

# print the time our boot was finished
date
