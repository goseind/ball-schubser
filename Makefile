BOT_IP=192.168.31.4

.PHONY: all simulator nav detect
all: get build install

help: ## Display this help screen
	@grep -h -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

sim: ## Launch turtlebot simulator
	export TURTLEBOT3_MODEL="burger"
	roslaunch turtlebot3_fake turtlebot3_fake.launch

bot:
	ssh ubuntu@$(BOT_IP)

bot-cam:
	ssh -R 11311:localhost:11311 ubuntu@$(BOT_IP) ./cam.sh

build:
	cd docker/ && docker build -t yolo-ros .
