all:
	export CAQTDM_DISPLAY_PATH=$(CAQTDM_DISPLAY_PATH):/APSshare/uisys/sr/id
	/APSshare/bin/adl2ui urRobot.adl
	/APSshare/bin/adl2ui dashboard.adl
	/APSshare/bin/adl2ui ur_rtde_control.adl
	mv urRobot.ui ../ui/autoconvert/
	mv dashboard.ui ../ui/autoconvert/
	mv ur_rtde_control.ui ../ui/autoconvert/
clean:
	rm ../ui/autoconvert/urRobot.ui
	rm ../ui/autoconvert/dashboard.ui
	rm ../ui/autoconvert/ur_rtde_control.ui
