scan_ports:
	uv run python -m lerobot.scripts.lerobot_find_port

setup:
	uv run python -m lerobot.scripts.lerobot_setup_motors $(SETUP_ARGS)

calibrate:
	uv run python -m lerobot.scripts.lerobot_calibrate $(CALIBRATE_ARGS)

teleop:
	uv run python -m lerobot.scripts.lerobot_teleoperate $(TELEOP_ARGS)
# Usage:
#   make setup SETUP_ARGS="--robot.type=so101_follower --robot.port=/dev/ttyACM0"
#   make calibrate CALIBRATE_ARGS="--robot.type=so101_follower --robot.port=/dev/ttyACM0"
# 	make teleop TELEOP_ARGS="--robot.type=so101_follower     --robot.port=/dev/ttyACM0 \
                             --robot.id=zandla_follower_arm     --teleop.type=so101_leader\
							 --teleop.port=/dev/ttyACM1     --teleop.id=zandla_leader_arm" 
