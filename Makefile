# RoboGame 2026 — top-level Makefile
# Mac: make setup / make test / make mock
# Pi:  make ros-build / make run (see README.md)

SHELL := /bin/bash
PYTHON ?= python3
VENV := .venv
PIP := $(VENV)/bin/pip
PY := $(VENV)/bin/python

.PHONY: setup test mock docker-build ros-build run lint clean check-config

## setup: create local venv and install dev dependencies (Mac friendly)
setup:
	$(PYTHON) -m venv $(VENV)
	$(PIP) install --upgrade pip
	$(PIP) install -e ".[dev]"

## test: run the full pytest suite (no hardware required)
test:
	$(PY) -m pytest

## mock: run a complete simulated match (no hardware required)
mock:
	$(PY) -m mock.runner

## lint: quick syntax check of core package
lint:
	$(PY) -m compileall -q core mock

## check-config: validate all config files
check-config:
	$(PY) scripts/check_config.py

## docker-build: build the ROS 2 Jazzy development container (arm64/amd64)
docker-build:
	docker build -f docker/Dockerfile.ros -t robogame-ros:dev .

## ros-build: compile the ROS 2 workspace inside Docker (no ROS needed on host)
ros-build: docker-build
	docker run --rm -v $$(pwd):/workspace -w /workspace robogame-ros:dev \
		bash -c "source /opt/ros/jazzy/setup.bash && cd ros2_ws && rosdep install --from-paths src --ignore-src -r -y || true && colcon build --symlink-install"

## run: launch the robot on the Raspberry Pi (requires ROS 2 on host)
run:
	bash scripts/start_robot.sh

clean:
	rm -rf $(VENV) build dist *.egg-info .pytest_cache
	rm -rf ros2_ws/build ros2_ws/install ros2_ws/log
	find . -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
