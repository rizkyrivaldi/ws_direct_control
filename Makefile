t1:
	@sudo ldconfig /usr/local/lib/
	@MicroXRCEAgent udp4 -p 8888 # Start agent

t2:
	@make -C ~/PX4-Autopilot px4_sitl_default gazebo-classic

t2_baylands:
	@make -C ~/PX4-Autopilot px4_sitl_default gazebo-classic_iris__baylands

t2_solo:
	@make -C ~/PX4-Autopilot px4_sitl gazebo-classic_solo

t3:
	@~/QGroundControl/QGroundControl.AppImage

s: #SHELL:=/bin/bash
	@bash -c "source install/setup.bash"