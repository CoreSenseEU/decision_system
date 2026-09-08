#!/bin/sh
export DEBIAN_FRONTEND=noninteractive
sudo apt-get update
sudo apt-get install -y python3-pip swi-prolog
pip install --user --break-system-packages pyswip vcstool
vcs import --recursive --input dependencies.repos ../
