t1:
	@sudo ldconfig /usr/local/lib/
	@MicroXRCEAgent udp4 -p 8888 # Start agent

t2:
	@make -C ~/PX4-Autopilot px4_sitl_default gazebo-classic

t3:
	@~/QGroundControl/QGroundControl.AppImage