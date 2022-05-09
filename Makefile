BOT_IP=192.168.43.251

.PHONY: all simulator navigation
all: get build install

sim: ## Launch turtlebot simulator
	export TURTLEBOT3_MODEL="burger"
	roslaunch turtlebot3_fake turtlebot3_fake.launch

nav: ## Launch navigation package
	rosrun ball_schubser_navigation control.py

bot:
	ssh -R 11311:localhost:11311 ubuntu@$(BOT_IP)

help: ## Display this help screen
	@grep -h -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'
