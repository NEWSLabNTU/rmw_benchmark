# Makefile for Autoware RMW Zenoh Benchmark
# Manages systemd services for Autoware planning simulator with different RMW implementations

# Get workspace directory (parent of this Makefile's directory)
WORKSPACE_DIR := $(shell cd "$(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))/.." && pwd)

# Local benchmark workspace directory
BENCHMARK_DIR := $(shell pwd)

# Service names
SERVICE_CYCLONEDDS := autoware-planning-cyclonedds
SERVICE_ZENOH := autoware-planning-zenoh
SERVICE_ROUTER := zenoh-router
SERVICE_STRESS_CYCLONEDDS := stress-test-cyclonedds
SERVICE_STRESS_ZENOH := stress-test-zenoh
SERVICE_STRESS_ROUTER := stress-test-zenoh-router

# Common parameters
DOMAIN_ID_CYCLONEDDS := 188
DOMAIN_ID_ZENOH := 189
DISPLAY := :1
MAP_PATH := $(HOME)/autoware_map/sample-map-planning
VEHICLE_MODEL := sample_vehicle
SENSOR_MODEL := sample_sensor_kit

.PHONY: help
help:
	@echo "Autoware RMW Zenoh Benchmark Makefile"
	@echo ""
	@echo "CycloneDDS targets:"
	@echo "  start-sim-cyclonedds     Start Autoware with CycloneDDS"
	@echo "  stop-sim-cyclonedds      Stop CycloneDDS simulation"
	@echo "  status-sim-cyclonedds    Show CycloneDDS service status"
	@echo "  logs-sim-cyclonedds      View CycloneDDS logs"
	@echo ""
	@echo "Zenoh targets:"
	@echo "  start-zenoh-router       Start Zenoh router daemon (required first)"
	@echo "  stop-zenoh-router        Stop Zenoh router"
	@echo "  status-zenoh-router      Show router status"
	@echo "  logs-zenoh-router        View router logs"
	@echo "  start-sim-zenoh          Start Autoware with Zenoh"
	@echo "  stop-sim-zenoh           Stop Zenoh simulation"
	@echo "  status-sim-zenoh         Show Zenoh service status"
	@echo "  logs-sim-zenoh           View Zenoh logs"
	@echo ""
	@echo "Build targets:"
	@echo "  build                       Build all packages in benchmark workspace"
	@echo ""
	@echo "Stress Test targets:"
	@echo "  build-stress-test           Build stress test package (alias for build)"
	@echo "  start-stress-cyclonedds     Start stress test with CycloneDDS (shared memory)"
	@echo "  stop-stress-cyclonedds      Stop CycloneDDS stress test"
	@echo "  status-stress-cyclonedds    Show CycloneDDS stress test status"
	@echo "  logs-stress-cyclonedds      View CycloneDDS stress test logs"
	@echo "  start-stress-zenoh-router   Start Zenoh router for stress test"
	@echo "  stop-stress-zenoh-router    Stop stress test Zenoh router"
	@echo "  status-stress-zenoh-router  Show stress test router status"
	@echo "  logs-stress-zenoh-router    View stress test router logs"
	@echo "  start-stress-zenoh          Start stress test with Zenoh (shared memory)"
	@echo "  stop-stress-zenoh           Stop Zenoh stress test"
	@echo "  status-stress-zenoh         Show Zenoh stress test status"
	@echo "  logs-stress-zenoh           View Zenoh stress test logs"
	@echo "  stop-stress-all             Stop all stress test services"
	@echo ""
	@echo "Management targets:"
	@echo "  stop-all                 Stop all services"
	@echo "  status-all               Show all service statuses"
	@echo "  help                     Show this help message"

.PHONY: start-sim-cyclonedds
start-sim-cyclonedds:
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

.PHONY: stop-sim-cyclonedds
stop-sim-cyclonedds:
	ros2 systemd stop $(SERVICE_CYCLONEDDS)

.PHONY: status-sim-cyclonedds
status-sim-cyclonedds:
	ros2 systemd status $(SERVICE_CYCLONEDDS)

.PHONY: logs-sim-cyclonedds
logs-sim-cyclonedds:
	ros2 systemd logs $(SERVICE_CYCLONEDDS)

.PHONY: start-zenoh-router
start-zenoh-router:
	ros2 systemd run \
		--name $(SERVICE_ROUTER) \
		--replace \
		--source $(WORKSPACE_DIR)/install/setup.bash \
		--source $(WORKSPACE_DIR)/rmw_zenoh_ws/install/setup.bash \
		--env RUST_LOG=zenoh=info \
		--env RMW_IMPLEMENTATION=rmw_zenoh_cpp \
		--description "Zenoh Router Daemon for RMW Zenoh" \
		rmw_zenoh_cpp rmw_zenohd

.PHONY: stop-zenoh-router
stop-zenoh-router:
	ros2 systemd stop $(SERVICE_ROUTER)

.PHONY: status-zenoh-router
status-zenoh-router:
	ros2 systemd status $(SERVICE_ROUTER)

.PHONY: logs-zenoh-router
logs-zenoh-router:
	ros2 systemd logs $(SERVICE_ROUTER)

.PHONY: start-sim-zenoh
start-sim-zenoh:
	@if ! systemctl --user is-active --quiet ros2-$(SERVICE_ROUTER).service 2>/dev/null; then \
		echo "ERROR: Zenoh router is not running. Start it first with: make start-zenoh-router"; \
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

.PHONY: stop-sim-zenoh
stop-sim-zenoh:
	ros2 systemd stop $(SERVICE_ZENOH)

.PHONY: status-sim-zenoh
status-sim-zenoh:
	ros2 systemd status $(SERVICE_ZENOH)

.PHONY: logs-sim-zenoh
logs-sim-zenoh:
	ros2 systemd logs $(SERVICE_ZENOH)

.PHONY: stop-all
stop-all:
	@for service in $(SERVICE_CYCLONEDDS) $(SERVICE_ZENOH) $(SERVICE_ROUTER) $(SERVICE_STRESS_CYCLONEDDS) $(SERVICE_STRESS_ZENOH) $(SERVICE_STRESS_ROUTER); do \
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
	@echo ""
	@echo "Stress Test Services:"
	@for service in $(SERVICE_STRESS_CYCLONEDDS) $(SERVICE_STRESS_ZENOH) $(SERVICE_STRESS_ROUTER); do \
		if systemctl --user is-active --quiet ros2-$$service.service 2>/dev/null; then \
			echo "  $$service: ACTIVE"; \
		else \
			echo "  $$service: INACTIVE"; \
		fi; \
	done

# ====================
# Stress Test Targets
# ====================

# Stress test configuration
STRESS_TEST_DIR := $(BENCHMARK_DIR)/src/stress_test
STRESS_WIDTH := 1920
STRESS_HEIGHT := 1080
STRESS_FPS := 30

.PHONY: build
build:
	@echo "Building benchmark workspace packages..."
	cd $(BENCHMARK_DIR) && \
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

.PHONY: build-stress-test
build-stress-test: build

.PHONY: start-stress-cyclonedds
start-stress-cyclonedds: build-stress-test
	@echo "Starting stress test with CycloneDDS + Shared Memory"
	@echo "Resolution: $(STRESS_WIDTH)x$(STRESS_HEIGHT) @ $(STRESS_FPS) FPS"
	ros2 systemd launch \
		--name $(SERVICE_STRESS_CYCLONEDDS) \
		--replace \
		--domain-id $(DOMAIN_ID_CYCLONEDDS) \
		--rmw rmw_cyclonedds_cpp \
		--source $(BENCHMARK_DIR)/install/setup.bash \
		--env CYCLONEDDS_URI=file://$(STRESS_TEST_DIR)/config/cyclonedds_shm.xml \
		--description "RMW Stress Test with CycloneDDS (Shared Memory)" \
		rmw_stress_test stress_test.launch.py \
		width:=$(STRESS_WIDTH) \
		height:=$(STRESS_HEIGHT) \
		fps:=$(STRESS_FPS)

.PHONY: stop-stress-cyclonedds
stop-stress-cyclonedds:
	ros2 systemd stop $(SERVICE_STRESS_CYCLONEDDS)

.PHONY: status-stress-cyclonedds
status-stress-cyclonedds:
	ros2 systemd status $(SERVICE_STRESS_CYCLONEDDS)

.PHONY: logs-stress-cyclonedds
logs-stress-cyclonedds:
	ros2 systemd logs $(SERVICE_STRESS_CYCLONEDDS)

.PHONY: start-stress-zenoh-router
start-stress-zenoh-router:
	@echo "Starting Zenoh router for stress test with shared memory..."
	ros2 systemd run \
		--name $(SERVICE_STRESS_ROUTER) \
		--replace \
		--source $(BENCHMARK_DIR)/install/setup.bash \
		--source $(WORKSPACE_DIR)/rmw_zenoh_ws/install/setup.bash \
		--env ZENOH_CONFIG=file://$(STRESS_TEST_DIR)/config/zenoh_shm.json5 \
		--env RUST_LOG=zenoh=info \
		--env RMW_IMPLEMENTATION=rmw_zenoh_cpp \
		--description "Zenoh Router for Stress Test (Shared Memory)" \
		rmw_zenoh_cpp rmw_zenohd

.PHONY: stop-stress-zenoh-router
stop-stress-zenoh-router:
	ros2 systemd stop $(SERVICE_STRESS_ROUTER)

.PHONY: status-stress-zenoh-router
status-stress-zenoh-router:
	ros2 systemd status $(SERVICE_STRESS_ROUTER)

.PHONY: logs-stress-zenoh-router
logs-stress-zenoh-router:
	ros2 systemd logs $(SERVICE_STRESS_ROUTER)

.PHONY: start-stress-zenoh
start-stress-zenoh: build-stress-test
	@if ! systemctl --user is-active --quiet ros2-$(SERVICE_STRESS_ROUTER).service 2>/dev/null; then \
		echo "ERROR: Zenoh router is not running. Start it first with: make start-stress-zenoh-router"; \
		exit 1; \
	fi
	@echo "Starting stress test with Zenoh + Shared Memory"
	@echo "Resolution: $(STRESS_WIDTH)x$(STRESS_HEIGHT) @ $(STRESS_FPS) FPS"
	ros2 systemd launch \
		--name $(SERVICE_STRESS_ZENOH) \
		--replace \
		--domain-id $(DOMAIN_ID_ZENOH) \
		--rmw rmw_zenoh_cpp \
		--source $(BENCHMARK_DIR)/install/setup.bash \
		--source $(WORKSPACE_DIR)/rmw_zenoh_ws/install/setup.bash \
		--env ZENOH_CONFIG=file://$(STRESS_TEST_DIR)/config/zenoh_shm.json5 \
		--description "RMW Stress Test with Zenoh (Shared Memory)" \
		rmw_stress_test stress_test.launch.py \
		width:=$(STRESS_WIDTH) \
		height:=$(STRESS_HEIGHT) \
		fps:=$(STRESS_FPS)

.PHONY: stop-stress-zenoh
stop-stress-zenoh:
	ros2 systemd stop $(SERVICE_STRESS_ZENOH)

.PHONY: status-stress-zenoh
status-stress-zenoh:
	ros2 systemd status $(SERVICE_STRESS_ZENOH)

.PHONY: logs-stress-zenoh
logs-stress-zenoh:
	ros2 systemd logs $(SERVICE_STRESS_ZENOH)

.PHONY: stop-stress-all
stop-stress-all:
	@for service in $(SERVICE_STRESS_CYCLONEDDS) $(SERVICE_STRESS_ZENOH) $(SERVICE_STRESS_ROUTER); do \
		if systemctl --user is-active --quiet ros2-$$service.service 2>/dev/null; then \
			echo "Stopping $$service"; \
			ros2 systemd stop $$service; \
		fi; \
	done
