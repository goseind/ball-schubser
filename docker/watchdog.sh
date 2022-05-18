#! /bin/bash

echo "Waiting a few seconds so roscore can start ..."
sleep 10s

echo "Starting '$1' with auto-restart on file change ..."
watchmedo auto-restart --directory=./ --pattern=*.py --recursive -- python3 $1
