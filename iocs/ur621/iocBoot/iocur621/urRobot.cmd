epicsEnvSet("ROBOT_NAME", "")

# Set up UR RTDE interface
epicsEnvSet("ASYN_PORT1", "ur_asyn1")
dbLoadRecords("$(TOP)/../../db/rtde_receive.db", "P=$(PREFIX), R=$(ROBOT_NAME), PORT=$(ASYN_PORT1), ADDR=0")
URRobotRTDEConfig("$(ASYN_PORT1)", "164.54.104.148")

# Set up UR dashboard server
epicsEnvSet("ASYN_PORT2", "ur_asyn2")
dbLoadRecords("$(TOP)/../../db/dashboard.db", "P=$(PREFIX), R=$(ROBOT_NAME), PORT=$(ASYN_PORT2), ADDR=0")
URRobotDashboardConfig("$(ASYN_PORT2)", "164.54.104.148")
