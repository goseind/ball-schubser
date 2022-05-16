BOT_IP=192.168.168.4

.PHONY: all simulator nav detect
all: get build install

help: ## Display this help screen
	@grep -h -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

sim: ## Launch turtlebot simulator
	export TURTLEBOT3_MODEL="burger"
	roslaunch turtlebot3_fake turtlebot3_fake.launch

bot:
	ssh -R 11311:localhost:11311 ubuntu@$(BOT_IP)

bot-cam:
	ssh -R 11311:localhost:11311 ubuntu@$(BOT_IP) ./cam.sh

build:
	cd docker/ && docker build -t yolo-ros .

detect:
	docker run --rm -ti -v $$(pwd)/detect:/app --device /dev/video0:/dev/video0 -e ROS_MASTER_URI=http://192.168.168.5:11311 yolo-ros /watchdog.sh detect.py

nav:
	docker run --rm -ti -v $$(pwd)/navigation:/app --device /dev/video0:/dev/video0 -e ROS_MASTER_URI=http://192.168.168.5:11311 yolo-ros /watchdog.sh navigation.py
