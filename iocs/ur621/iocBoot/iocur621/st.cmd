# ../../bin/${EPICS_HOST_ARCH}/ur621 st.cmd
< envPaths

epicsEnvSet("PREFIX", "ur621:")

dbLoadDatabase("../../dbd/iocur621Linux.dbd")
iocur621Linux_registerRecordDeviceDriver(pdbbase)

< urRobot.cmd

###############################################################################
iocInit
###############################################################################

# print the time our boot was finished
date
