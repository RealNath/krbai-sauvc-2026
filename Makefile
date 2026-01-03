all: run

.PHONY: all esp32-build esp32-monitor esp32-reset webserver build run attach

ESP32_SRC ?= ./esp32_firmware
ESP32_PORT ?= /dev/ttyACM0
ESP32_FQBN ?= esp32:esp32:esp32s3
ESP32_BAUDRATE ?= 115200

ROS2_WS ?= ./ros2_ws
ROS2_CONTAINER_TAG ?= ros-humble-core-dev

MISSION_FILE ?= mission.yaml

SCRIPT_SRC ?= ./scripts
CAMERA_SERVICE_OPTION ?= --log

esp32-build:
	@echo -n "[Makefile]: "
	arduino-cli compile \
		--fqbn $(ESP32_FQBN) \
		--output-dir $(ESP32_SRC)/build \
		$(ESP32_SRC)/main_controller/

	arduino-cli upload \
		--fqbn $(ESP32_FQBN) \
		--port $(ESP32_PORT) \
		--build-path $(ESP32_SRC)/build


esp32-monitor:
	@echo -n "[Makefile]: "
	arduino-cli monitor \
		--port $(ESP32_PORT) \
		--fqbn $(ESP32_FQBN) \
		--config baudrate=$(ESP32_BAUDRATE)


esp32-reset:
	@echo -n "[Makefile]: "
	@set -e; \
	if lsof $(ESP32_PORT) >/dev/null 2>&1 || fuser $(ESP32_PORT) >/dev/null 2>&1; then \
		echo "Port $(ESP32_PORT) is busy, close serial monitor or other process which uses this port first before resetting."; \
		exit 1; \
	fi; \
	esptool --chip esp32 \
		--port $(ESP32_PORT) \
		--no-stub flash_id


webserver:
	@echo -n "[Makefile]: "
	python3 $(SCRIPT_SRC)/camera_service.py $(CAMERA_SERVICE_OPTION)


build:
	@echo -n "[Makefile]: "
	sudo docker build --tag "$(ROS2_CONTAINER_TAG)" .


run: build
	@echo -n "[Makefile]: "
	-sudo docker run \
		--interactive \
		--tty \
		--rm \
		--device $(ESP32_PORT):$(ESP32_PORT) \
		--volume $(ROS2_WS):/home/krbai-sauvc-2026/$(ROS2_WS) \
		$(ROS2_CONTAINER_TAG):latest $(ESP32_PORT) $(MISSION_FILE)


attach:
	@echo -n "[Makefile]: "
	@set -e; \
	CONTAINER_ID=$$(sudo docker ps -q | head -n 1); \
	test -n "$$CONTAINER_ID" || { \
		echo "No running ROS2 containers found, make sure setup is correct"; exit 1; \
	}; \
	sudo docker attach "$$CONTAINER_ID"

