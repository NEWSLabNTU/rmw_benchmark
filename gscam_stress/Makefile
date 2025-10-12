# Makefile for gscam Stress Test
# Manages stress test with CycloneDDS and Zenoh using gscam publisher

SHELL := /bin/bash

# Get directories
GSCAM_STRESS_DIR := $(shell pwd)
MAKEFILE_DIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))
BENCHMARK_ROOT := $(shell cd "$(MAKEFILE_DIR)/.." && pwd)
RMW_ZENOH_INSTALL := $(BENCHMARK_ROOT)/common/rmw_zenoh/install

# Service names
SERVICE_CYCLONEDDS := stress-test-cyclonedds
SERVICE_ZENOH := stress-test-zenoh
SERVICE_ROUTER := stress-test-zenoh-router

# Common parameters
DOMAIN_ID_CYCLONEDDS := 188
DOMAIN_ID_ZENOH := 189

# Stress test configuration
STRESS_TEST_DIR := $(GSCAM_STRESS_DIR)/src/stress_test
STRESS_WIDTH := 1920
STRESS_HEIGHT := 1080
STRESS_FPS := 30

.PHONY: help
help:
	@echo "gscam Stress Test Makefile"
	@echo ""
	@echo "Build targets:"
	@echo "  build                Build stress test package"
	@echo "  clean                Clean build artifacts"
	@echo ""
	@echo "CycloneDDS Stress Test:"
	@echo "  start-cyclonedds     Start stress test with CycloneDDS"
	@echo "  stop-cyclonedds      Stop CycloneDDS stress test"
	@echo "  status-cyclonedds    Show CycloneDDS stress test status"
	@echo "  logs-cyclonedds      View CycloneDDS stress test logs"
	@echo ""
	@echo "Zenoh Stress Test:"
	@echo "  start-router         Start Zenoh router for stress test"
	@echo "  stop-router          Stop stress test Zenoh router"
	@echo "  status-router        Show stress test router status"
	@echo "  logs-router          View stress test router logs"
	@echo "  start-zenoh          Start stress test with Zenoh"
	@echo "  stop-zenoh           Stop Zenoh stress test"
	@echo "  status-zenoh         Show Zenoh stress test status"
	@echo "  logs-zenoh           View Zenoh stress test logs"
	@echo ""
	@echo "Management targets:"
	@echo "  stop-all             Stop all stress test services"
	@echo "  status-all           Show all service statuses"

.PHONY: build
build:
	@echo "Building stress test package..."
	cd $(GSCAM_STRESS_DIR) && \
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

.PHONY: clean
clean:
	rm -rf build install log

.PHONY: start-cyclonedds
start-cyclonedds: build
	@echo "Starting stress test with CycloneDDS"
	@echo "Resolution: $(STRESS_WIDTH)x$(STRESS_HEIGHT) @ $(STRESS_FPS) FPS"
	@. $(GSCAM_STRESS_DIR)/install/setup.bash && \
	ros2 systemd launch \
		--name $(SERVICE_CYCLONEDDS) \
		--replace \
		--domain-id $(DOMAIN_ID_CYCLONEDDS) \
		--rmw rmw_cyclonedds_cpp \
		--source $(GSCAM_STRESS_DIR)/install/setup.bash \
		--env CYCLONEDDS_URI=file://$(STRESS_TEST_DIR)/config/cyclonedds_shm.xml \
		--description "RMW Stress Test with CycloneDDS" \
		rmw_stress_test stress_test.launch.py \
		width:=$(STRESS_WIDTH) \
		height:=$(STRESS_HEIGHT) \
		fps:=$(STRESS_FPS)

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
	@echo "Starting Zenoh router for stress test with shared memory..."
	@. $(GSCAM_STRESS_DIR)/install/setup.bash && \
	. $(RMW_ZENOH_INSTALL)/setup.bash && \
	ros2 systemd run \
		--name $(SERVICE_ROUTER) \
		--replace \
		--source $(GSCAM_STRESS_DIR)/install/setup.bash \
		--source $(RMW_ZENOH_INSTALL)/setup.bash \
		--env ZENOH_CONFIG=file://$(STRESS_TEST_DIR)/config/zenoh_shm.json5 \
		--env RUST_LOG=zenoh=info \
		--env RMW_IMPLEMENTATION=rmw_zenoh_cpp \
		--description "Zenoh Router for Stress Test (Shared Memory)" \
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
start-zenoh: build
	@if ! systemctl --user is-active --quiet ros2-$(SERVICE_ROUTER).service 2>/dev/null; then \
		echo "ERROR: Zenoh router is not running. Start it first with: make start-router"; \
		exit 1; \
	fi
	@echo "Starting stress test with Zenoh + Shared Memory"
	@echo "Resolution: $(STRESS_WIDTH)x$(STRESS_HEIGHT) @ $(STRESS_FPS) FPS"
	@. $(GSCAM_STRESS_DIR)/install/setup.bash && \
	. $(RMW_ZENOH_INSTALL)/setup.bash && \
	ros2 systemd launch \
		--name $(SERVICE_ZENOH) \
		--replace \
		--domain-id $(DOMAIN_ID_ZENOH) \
		--rmw rmw_zenoh_cpp \
		--source $(GSCAM_STRESS_DIR)/install/setup.bash \
		--source $(RMW_ZENOH_INSTALL)/setup.bash \
		--env ZENOH_CONFIG=file://$(STRESS_TEST_DIR)/config/zenoh_shm.json5 \
		--description "RMW Stress Test with Zenoh (Shared Memory)" \
		rmw_stress_test stress_test.launch.py \
		width:=$(STRESS_WIDTH) \
		height:=$(STRESS_HEIGHT) \
		fps:=$(STRESS_FPS)

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
	@echo "Stress Test Services:"
	@for service in $(SERVICE_CYCLONEDDS) $(SERVICE_ZENOH) $(SERVICE_ROUTER); do \
		if systemctl --user is-active --quiet ros2-$$service.service 2>/dev/null; then \
			echo "  $$service: ACTIVE"; \
		else \
			echo "  $$service: INACTIVE"; \
		fi; \
	done
