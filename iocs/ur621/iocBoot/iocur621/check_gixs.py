from epics import caget, caput
import time
#  "bcur:Sample$(BlockID):Exists",
MAX_BLOCK = 24

pvs = [
    "bcur:Sample$(BlockID):InReservoir",
    "bcur:Sample$(BlockID):InGripper",
    "bcur:Sample$(BlockID):InSampleStage",
    "bcur:Sample$(BlockID):Measured",
]

for i in range(1, MAX_BLOCK+1):
    for pv in pvs:
        pv_sub = pv.replace("$(BlockID)", str(i))
        caput(pv_sub, 1)
        time.sleep(0.1)
        if "InSampleStage" not in pv_sub and "Measured" not in pv_sub:
            caput(pv_sub, 0)

for i in range(1, MAX_BLOCK+1):
    for pv in pvs:
        pv_sub = pv.replace("$(BlockID)", str(i))
        caput(pv_sub, 0)
        caput(f"bcur:Sample{i}:Exists", 0)

