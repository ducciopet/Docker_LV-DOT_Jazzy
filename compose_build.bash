#!/bin/bash

# Allow X11 connections from Docker containers
xhost +local:docker

docker compose build
