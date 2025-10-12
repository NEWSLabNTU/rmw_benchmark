# Makefile for Autoware Planning Simulation
# Manages systemd services for Autoware planning simulator with different RMW implementations

SHELL := /bin/bash

# Get workspace directory (two levels up from this Makefile)
WORKSPACE_DIR := $(shell cd "$(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))/../.." && pwd)

# Service names
SERVICE_CYCLONEDDS := autoware-planning-cyclonedds
SERVICE_ZENOH := autoware-planning-zenoh
SERVICE_ROUTER := zenoh-router

# Common parameters
DOMAIN_ID_CYCLONEDDS := 188
DOMAIN_ID_ZENOH := 189
DISPLAY := :1
MAP_PATH := $(HOME)/autoware_map/sample-map-planning
VEHICLE_MODEL := sample_vehicle
SENSOR_MODEL := sample_sensor_kit

.PHONY: help
help:
	@echo "Autoware Planning Simulation Makefile"
	@echo ""
	@echo "CycloneDDS targets:"
	@echo "  start-cyclonedds     Start Autoware with CycloneDDS"
	@echo "  stop-cyclonedds      Stop CycloneDDS simulation"
	@echo "  status-cyclonedds    Show CycloneDDS service status"
	@echo "  logs-cyclonedds      View CycloneDDS logs"
	@echo ""
	@echo "Zenoh targets:"
	@echo "  start-router         Start Zenoh router daemon (required first)"
	@echo "  stop-router          Stop Zenoh router"
	@echo "  status-router        Show router status"
	@echo "  logs-router          View router logs"
	@echo "  start-zenoh          Start Autoware with Zenoh"
	@echo "  stop-zenoh           Stop Zenoh simulation"
	@echo "  status-zenoh         Show Zenoh service status"
	@echo "  logs-zenoh           View Zenoh logs"
	@echo ""
	@echo "Management targets:"
	@echo "  stop-all             Stop all services"
	@echo "  status-all           Show all service statuses"

.PHONY: start-cyclonedds
start-cyclonedds:
	ros2 systemd launch \
		--name $(SERVICE_CYCLONEDDS) \
		--replace \
		--domain-id $(DOMAIN_ID_CYCLONEDDS) \
		--rmw rmw_cyclonedds_cpp \
		--source $(WORKSPACE_DIR)/install/setup.bash \
		--env DISPLAY=$(DISPLAY) \
		--description "Autoware Planning Simulator with CycloneDDS" \
		autoware_launch planning_simulator.launch.xml \
		map_path:=$(MAP_PATH) \
		vehicle_model:=$(VEHICLE_MODEL) \
		sensor_model:=$(SENSOR_MODEL)

.PHONY: stop-cyclonedds
stop-cyclonedds:
	ros2 systemd stop $(SERVICE_CYCLONEDDS)

.PHONY: status-cyclonedds
status-cyclonedds:
	ros2 systemd status $(SERVICE_CYCLONEDDS)

.PHONY: logs-cyclonedds
logs-cyclonedds:
	ros2 systemd logs $(SERVICE_CYCLONEDDS)

.PHONY: start-router
start-router:
	ros2 systemd run \
		--name $(SERVICE_ROUTER) \
		--replace \
		--source $(WORKSPACE_DIR)/install/setup.bash \
		--source $(WORKSPACE_DIR)/rmw_zenoh_ws/install/setup.bash \
		--env RUST_LOG=zenoh=info \
		--env RMW_IMPLEMENTATION=rmw_zenoh_cpp \
		--description "Zenoh Router Daemon for RMW Zenoh" \
		rmw_zenoh_cpp rmw_zenohd

.PHONY: stop-router
stop-router:
	ros2 systemd stop $(SERVICE_ROUTER)

.PHONY: status-router
status-router:
	ros2 systemd status $(SERVICE_ROUTER)

.PHONY: logs-router
logs-router:
	ros2 systemd logs $(SERVICE_ROUTER)

.PHONY: start-zenoh
start-zenoh:
	@if ! systemctl --user is-active --quiet ros2-$(SERVICE_ROUTER).service 2>/dev/null; then \
		echo "ERROR: Zenoh router is not running. Start it first with: make start-router"; \
		exit 1; \
	fi
	ros2 systemd launch \
		--name $(SERVICE_ZENOH) \
		--replace \
		--domain-id $(DOMAIN_ID_ZENOH) \
		--rmw rmw_zenoh_cpp \
		--source $(WORKSPACE_DIR)/install/setup.bash \
		--source $(WORKSPACE_DIR)/rmw_zenoh_ws/install/setup.bash \
		--env DISPLAY=$(DISPLAY) \
		--description "Autoware Planning Simulator with Zenoh" \
		autoware_launch planning_simulator.launch.xml \
		map_path:=$(MAP_PATH) \
		vehicle_model:=$(VEHICLE_MODEL) \
		sensor_model:=$(SENSOR_MODEL)

.PHONY: stop-zenoh
stop-zenoh:
	ros2 systemd stop $(SERVICE_ZENOH)

.PHONY: status-zenoh
status-zenoh:
	ros2 systemd status $(SERVICE_ZENOH)

.PHONY: logs-zenoh
logs-zenoh:
	ros2 systemd logs $(SERVICE_ZENOH)

.PHONY: stop-all
stop-all:
	@for service in $(SERVICE_CYCLONEDDS) $(SERVICE_ZENOH) $(SERVICE_ROUTER); do \
		if systemctl --user is-active --quiet ros2-$$service.service 2>/dev/null; then \
			echo "Stopping $$service"; \
			ros2 systemd stop $$service; \
		fi; \
	done

.PHONY: status-all
status-all:
	@echo "Planning Simulation Services:"
	@for service in $(SERVICE_CYCLONEDDS) $(SERVICE_ZENOH) $(SERVICE_ROUTER); do \
		if systemctl --user is-active --quiet ros2-$$service.service 2>/dev/null; then \
			echo "  $$service: ACTIVE"; \
		else \
			echo "  $$service: INACTIVE"; \
		fi; \
	done
