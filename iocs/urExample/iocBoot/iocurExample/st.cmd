# ../../bin/${EPICS_HOST_ARCH}/urExample st.cmd
< envPaths

epicsEnvSet("PREFIX", "urExample:")

dbLoadDatabase("../../dbd/iocurExampleLinux.dbd")
iocurExampleLinux_registerRecordDeviceDriver(pdbbase)

# load the urRobot iocsh example
iocshLoad("$(URROBOT)/iocsh/urRobot.iocsh", "PREFIX=$(PREFIX), IP=164.54.104.148")

###############################################################################
iocInit
###############################################################################

# print the time our boot was finished
date
