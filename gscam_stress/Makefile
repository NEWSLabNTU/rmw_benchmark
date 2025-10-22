# Makefile for gscam Stress Test
# Manages stress test with CycloneDDS and Zenoh using gscam publisher

SHELL := /bin/bash

# Get directories
GSCAM_STRESS_DIR := $(shell pwd)
MAKEFILE_DIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))
BENCHMARK_ROOT := $(shell cd "$(MAKEFILE_DIR)/.." && pwd)

# Service names
SERVICE_CYCLONEDDS := stress-test-cyclonedds
SERVICE_ZENOH := stress-test-zenoh
SERVICE_ROUTER := stress-test-zenoh-router

# Common parameters
DOMAIN_ID_CYCLONEDDS := 188
DOMAIN_ID_ZENOH := 189

# Stress test configuration
STRESS_TEST_DIR := $(GSCAM_STRESS_DIR)/src/stress_test
CONFIG_DIR := $(STRESS_TEST_DIR)/config
RMW_CONFIG_DIR := $(GSCAM_STRESS_DIR)/rmw_config

# Select config file (can be overridden: make STRESS_CONFIG=high.conf start-cyclonedds)
STRESS_CONFIG ?= medium.conf
STRESS_CONFIG_PATH := $(CONFIG_DIR)/$(STRESS_CONFIG)

# Helper function to read config value
define read_config
$(shell grep -E "^$(1)=" $(STRESS_CONFIG_PATH) 2>/dev/null | cut -d'=' -f2- | sed 's/^"\(.*\)"$$/\1/' | sed "s/^'\(.*\)'$$/\1/")
endef

# Load configuration from config file
ifneq (,$(wildcard $(STRESS_CONFIG_PATH)))
    # Video parameters
    WIDTH := $(call read_config,WIDTH)
    HEIGHT := $(call read_config,HEIGHT)
    FPS := $(call read_config,FPS)

    # GStreamer pipeline with variable substitution
    GSTREAMER_PIPELINE_RAW := $(call read_config,GSTREAMER_PIPELINE)
    GSTREAMER_PIPELINE := $(subst {WIDTH},$(WIDTH),$(subst {HEIGHT},$(HEIGHT),$(subst {FPS},$(FPS),$(GSTREAMER_PIPELINE_RAW))))

    # QoS parameters
    QOS_RELIABILITY := $(call read_config,QOS_RELIABILITY)
    QOS_DURABILITY := $(call read_config,QOS_DURABILITY)
    QOS_HISTORY := $(call read_config,QOS_HISTORY)
    QOS_HISTORY_DEPTH := $(call read_config,QOS_HISTORY_DEPTH)
    QOS_DEADLINE_MS := $(call read_config,QOS_DEADLINE_MS)
    QOS_LIVELINESS := $(call read_config,QOS_LIVELINESS)
    QOS_LIVELINESS_LEASE_MS := $(call read_config,QOS_LIVELINESS_LEASE_MS)

    # Fallback to defaults if values are missing
    WIDTH ?= 1280
    HEIGHT ?= 720
    FPS ?= 30
    GSTREAMER_PIPELINE ?= videotestsrc pattern=smpte ! video/x-raw,width=$(WIDTH),height=$(HEIGHT),framerate=$(FPS)/1 ! videoconvert
    QOS_RELIABILITY ?= RELIABLE
    QOS_DURABILITY ?= VOLATILE
    QOS_HISTORY ?= KEEP_LAST
    QOS_HISTORY_DEPTH ?= 10
    QOS_DEADLINE_MS ?= 0
    QOS_LIVELINESS ?= AUTOMATIC
    QOS_LIVELINESS_LEASE_MS ?= 0
else
    # Default values if config file doesn't exist
    $(warning Config file not found: $(STRESS_CONFIG_PATH))
    $(warning Using default medium configuration)
    WIDTH := 1280
    HEIGHT := 720
    FPS := 30
    GSTREAMER_PIPELINE := videotestsrc pattern=smpte ! video/x-raw,width=$(WIDTH),height=$(HEIGHT),framerate=$(FPS)/1 ! videoconvert
    QOS_RELIABILITY := RELIABLE
    QOS_DURABILITY := VOLATILE
    QOS_HISTORY := KEEP_LAST
    QOS_HISTORY_DEPTH := 10
    QOS_DEADLINE_MS := 0
    QOS_LIVELINESS := AUTOMATIC
    QOS_LIVELINESS_LEASE_MS := 0
endif

# Extract profile name from config filename (e.g., medium.conf -> medium)
PROFILE_NAME := $(basename $(STRESS_CONFIG))

.PHONY: help
help:
	@echo "gscam Stress Test Makefile"
	@echo ""
	@echo "Current Configuration:"
	@echo "  Config: $(STRESS_CONFIG) ($(PROFILE_NAME))"
	@echo "  Resolution: $(WIDTH)x$(HEIGHT) @ $(FPS) FPS"
	@$(eval DATA_RATE_MB := $(shell echo "scale=1; $(WIDTH) * $(HEIGHT) * 3 * $(FPS) / 1048576" | bc 2>/dev/null || echo "N/A"))
	@echo "  Data rate: ~$(DATA_RATE_MB) MB/s"
	@echo ""
	@echo "Build targets:"
	@echo "  build                Build stress test package"
	@echo "  clean                Clean build artifacts"
	@echo ""
	@echo "Configuration:"
	@echo "  show-config          Show detailed configuration"
	@echo "  list-configs         List available configuration files"
	@echo ""
	@echo "To use a different config:"
	@echo "  make STRESS_CONFIG=high.conf start-cyclonedds"
	@echo ""
	@echo "CycloneDDS Stress Test:"
	@echo "  start-cyclonedds-router  Start Iceoryx RouDi daemon (required for shared memory)"
	@echo "  stop-cyclonedds-router   Stop Iceoryx RouDi daemon"
	@echo "  status-cyclonedds-router Show RouDi daemon status"
	@echo "  logs-cyclonedds-router   View RouDi daemon logs"
	@echo "  start-cyclonedds-test    Start stress test with CycloneDDS"
	@echo "  stop-cyclonedds-test     Stop CycloneDDS stress test"
	@echo "  status-cyclonedds-test   Show CycloneDDS stress test status"
	@echo "  logs-cyclonedds-test     View CycloneDDS stress test logs"
	@echo ""
	@echo "Zenoh Stress Test:"
	@echo "  start-zenoh-router   Start Zenoh router for stress test"
	@echo "  stop-zenoh-router    Stop Zenoh router"
	@echo "  status-zenoh-router  Show Zenoh router status"
	@echo "  logs-zenoh-router    View Zenoh router logs"
	@echo "  start-zenoh-test     Start stress test with Zenoh"
	@echo "  stop-zenoh-test      Stop Zenoh stress test"
	@echo "  status-zenoh-test    Show Zenoh stress test status"
	@echo "  logs-zenoh-test      View Zenoh stress test logs"
	@echo ""
	@echo "Management targets:"
	@echo "  stop-all             Stop all stress test services"
	@echo "  status-all           Show all service statuses"
	@echo ""
	@echo "Benchmark targets:"
	@echo "  benchmark            Run complete benchmark suite (14 configs, ~35 min)"
	@echo "  benchmark-quick      Run quick benchmark (6 configs, ~8 min)"
	@echo "  benchmark-analyze    Analyze latest benchmark results"

.PHONY: show-config
show-config:
	@echo "======================================================================"
	@echo "Stress Test Configuration"
	@echo "======================================================================"
	@echo "Configuration File: $(STRESS_CONFIG)"
	@echo "Profile Name: $(PROFILE_NAME)"
	@echo "Full Path: $(STRESS_CONFIG_PATH)"
	@echo ""
	@echo "Video Parameters:"
	@echo "  Width:  $(WIDTH) pixels"
	@echo "  Height: $(HEIGHT) pixels"
	@echo "  FPS:    $(FPS) frames/sec"
	@echo ""
	@echo "Estimated Data Rate (uncompressed RGB):"
	@$(eval DATA_RATE_MB := $(shell echo "scale=1; $(WIDTH) * $(HEIGHT) * 3 * $(FPS) / 1048576" | bc 2>/dev/null || echo "N/A"))
	@echo "  ~$(DATA_RATE_MB) MB/s"
	@echo ""
	@echo "GStreamer Pipeline:"
	@echo "  $(GSTREAMER_PIPELINE)"
	@echo ""
	@echo "QoS (Quality of Service) Settings:"
	@echo "  Reliability:       $(QOS_RELIABILITY)"
	@echo "  Durability:        $(QOS_DURABILITY)"
	@echo "  History:           $(QOS_HISTORY)"
	@echo "  History Depth:     $(QOS_HISTORY_DEPTH)"
	@echo "  Deadline:          $(QOS_DEADLINE_MS) ms"
	@echo "  Liveliness:        $(QOS_LIVELINESS)"
	@echo "  Liveliness Lease:  $(QOS_LIVELINESS_LEASE_MS) ms"
	@echo ""
	@echo "RMW Configuration Files:"
	@echo "  CycloneDDS: $(RMW_CONFIG_DIR)/cyclonedds_shm.xml"
	@echo "  Zenoh:      $(RMW_CONFIG_DIR)/zenoh_shm.json5"

.PHONY: list-configs
list-configs:
	@echo "Available configuration files in $(CONFIG_DIR):"
	@echo ""
	@for conf in $(CONFIG_DIR)/*.conf; do \
		if [ -f "$$conf" ]; then \
			filename=$$(basename $$conf); \
			if [ "$$filename" = "$(STRESS_CONFIG)" ]; then \
				echo "  * $$filename (current)"; \
			else \
				echo "    $$filename"; \
			fi; \
		fi; \
	done
	@echo ""
	@echo "Example templates (copy and modify):"
	@for example in $(CONFIG_DIR)/*.example; do \
		if [ -f "$$example" ]; then \
			echo "    $$(basename $$example)"; \
		fi; \
	done
	@echo ""
	@echo "To use a config: make STRESS_CONFIG=filename start-cyclonedds"

.PHONY: build
build:
	@echo "Building stress test package..."
	@. /opt/ros/humble/setup.sh && \
	cd $(GSCAM_STRESS_DIR) && \
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

.PHONY: clean
clean:
	rm -rf build install log

.PHONY: start-cyclonedds-test
start-cyclonedds-test:
	@echo "======================================================================"
	@echo "Starting stress test with CycloneDDS"
	@echo "======================================================================"
	@echo "Config: $(STRESS_CONFIG) ($(PROFILE_NAME))"
	@echo "Resolution: $(WIDTH)x$(HEIGHT) @ $(FPS) FPS"
	@$(eval DATA_RATE_MB := $(shell echo "scale=1; $(WIDTH) * $(HEIGHT) * 3 * $(FPS) / 1048576" | bc 2>/dev/null || echo "N/A"))
	@echo "Estimated data rate: ~$(DATA_RATE_MB) MB/s (uncompressed)"
	@echo "QoS: $(QOS_RELIABILITY)/$(QOS_DURABILITY)"
	@echo "======================================================================"
	@mkdir -p $(GSCAM_STRESS_DIR)/cyclonedds
	@$(eval CSV_DATETIME := $(shell date +%Y-%m-%d_%H-%M-%S))
	@$(eval CSV_PATH := $(GSCAM_STRESS_DIR)/cyclonedds/$(CSV_DATETIME).csv)
	@echo "CSV output: $(CSV_PATH)"
	@. /opt/ros/humble/setup.sh && \
	. $(GSCAM_STRESS_DIR)/install/setup.sh && \
	ros2 systemd launch \
		--name $(SERVICE_CYCLONEDDS) \
		--replace \
		--domain-id $(DOMAIN_ID_CYCLONEDDS) \
		--rmw rmw_cyclonedds_cpp \
		--source $(GSCAM_STRESS_DIR)/install/setup.sh \
		--env CYCLONEDDS_URI=file://$(RMW_CONFIG_DIR)/cyclonedds_shm.xml \
		--description 'RMW Stress Test with CycloneDDS ($(PROFILE_NAME))' \
		rmw_stress_test stress_test.launch.py \
		width:=$(WIDTH) \
		height:=$(HEIGHT) \
		fps:=$(FPS) \
		csv_output_path:=$(CSV_PATH) \
		qos_reliability:=$(QOS_RELIABILITY) \
		qos_durability:=$(QOS_DURABILITY) \
		qos_history:=$(QOS_HISTORY) \
		qos_history_depth:=$(QOS_HISTORY_DEPTH) \
		qos_deadline_ms:=$(QOS_DEADLINE_MS) \
		qos_liveliness:=$(QOS_LIVELINESS) \
		qos_liveliness_lease_ms:=$(QOS_LIVELINESS_LEASE_MS)

.PHONY: stop-cyclonedds-test
stop-cyclonedds-test:
	@. /opt/ros/humble/setup.sh && ros2 systemd stop $(SERVICE_CYCLONEDDS)

.PHONY: status-cyclonedds-test
status-cyclonedds-test:
	@. /opt/ros/humble/setup.sh && ros2 systemd status $(SERVICE_CYCLONEDDS)

.PHONY: logs-cyclonedds-test
logs-cyclonedds-test:
	@. /opt/ros/humble/setup.sh && ros2 systemd logs $(SERVICE_CYCLONEDDS)

# CycloneDDS Iceoryx RouDi daemon management
# Required for shared memory support
.PHONY: start-cyclonedds-router
start-cyclonedds-router:
	@echo "Starting Iceoryx RouDi daemon for CycloneDDS shared memory..."
	@mkdir -p $(GSCAM_STRESS_DIR)/cyclonedds
	@if pgrep -x "iox-roudi" > /dev/null; then \
		echo "Iceoryx RouDi already running (PID: $$(pgrep -x iox-roudi))"; \
	else \
		if [ -x "/opt/ros/humble/bin/iox-roudi" ]; then \
			. /opt/ros/humble/setup.sh && \
			/opt/ros/humble/bin/iox-roudi -c $(GSCAM_STRESS_DIR)/cyclonedds/roudi_config.toml > $(GSCAM_STRESS_DIR)/cyclonedds/roudi.log 2>&1 & \
			sleep 2; \
			if pgrep -x "iox-roudi" > /dev/null; then \
				ROUDI_PID=$$(pgrep -x iox-roudi); \
				echo $$ROUDI_PID > $(GSCAM_STRESS_DIR)/cyclonedds/roudi.pid; \
				echo "Iceoryx RouDi started (PID: $$ROUDI_PID)"; \
				echo "Log: $(GSCAM_STRESS_DIR)/cyclonedds/roudi.log"; \
			else \
				echo "ERROR: Failed to start Iceoryx RouDi"; \
				echo "Check log: $(GSCAM_STRESS_DIR)/cyclonedds/roudi.log"; \
				exit 1; \
			fi; \
		else \
			echo "ERROR: Iceoryx RouDi not found at /opt/ros/humble/bin/iox-roudi"; \
			echo "Install with: sudo apt install ros-humble-iceoryx-posh ros-humble-iceoryx-binding-c"; \
			exit 1; \
		fi; \
	fi

.PHONY: stop-cyclonedds-router
stop-cyclonedds-router:
	@echo "Stopping Iceoryx RouDi daemon..."
	@if [ -f "$(GSCAM_STRESS_DIR)/cyclonedds/roudi.pid" ]; then \
		pkill -x iox-roudi && echo "Iceoryx RouDi stopped" || echo "No RouDi process running"; \
		rm -f $(GSCAM_STRESS_DIR)/cyclonedds/roudi.pid; \
	else \
		pkill -x iox-roudi && echo "Iceoryx RouDi stopped" || echo "No RouDi process running"; \
	fi

.PHONY: status-cyclonedds-router
status-cyclonedds-router:
	@if pgrep -x "iox-roudi" > /dev/null; then \
		ROUDI_PID=$$(pgrep -x iox-roudi); \
		echo "Iceoryx RouDi: RUNNING (PID: $$ROUDI_PID)"; \
		if [ -f "$(GSCAM_STRESS_DIR)/cyclonedds/roudi.pid" ]; then \
			echo "PID file: $(GSCAM_STRESS_DIR)/cyclonedds/roudi.pid"; \
		fi; \
		if [ -f "$(GSCAM_STRESS_DIR)/cyclonedds/roudi.log" ]; then \
			echo "Log file: $(GSCAM_STRESS_DIR)/cyclonedds/roudi.log"; \
		fi; \
	else \
		echo "Iceoryx RouDi: NOT RUNNING"; \
	fi

.PHONY: logs-cyclonedds-router
logs-cyclonedds-router:
	@if [ -f "$(GSCAM_STRESS_DIR)/cyclonedds/roudi.log" ]; then \
		tail -50 $(GSCAM_STRESS_DIR)/cyclonedds/roudi.log; \
	else \
		echo "No RouDi log file found at $(GSCAM_STRESS_DIR)/cyclonedds/roudi.log"; \
	fi

.PHONY: start-zenoh-router
start-zenoh-router:
	@echo "Starting Zenoh router for stress test with shared memory..."
	@. /opt/ros/humble/setup.sh && \
	. $(GSCAM_STRESS_DIR)/install/setup.sh && \
	ros2 systemd run \
		--name $(SERVICE_ROUTER) \
		--replace \
		--source $(GSCAM_STRESS_DIR)/install/setup.sh \
		--env ZENOH_CONFIG=file://$(RMW_CONFIG_DIR)/zenoh_shm.json5 \
		--env RUST_LOG=zenoh=info \
		--env RMW_IMPLEMENTATION=rmw_zenoh_cpp \
		--description 'Zenoh Router for Stress Test (Shared Memory)' \
		rmw_zenoh_cpp rmw_zenohd

.PHONY: stop-zenoh-router
stop-zenoh-router:
	@. /opt/ros/humble/setup.sh && ros2 systemd stop $(SERVICE_ROUTER)

.PHONY: status-zenoh-router
status-zenoh-router:
	@. /opt/ros/humble/setup.sh && ros2 systemd status $(SERVICE_ROUTER)

.PHONY: logs-zenoh-router
logs-zenoh-router:
	@. /opt/ros/humble/setup.sh && ros2 systemd logs $(SERVICE_ROUTER)

.PHONY: start-zenoh-test
start-zenoh-test:
	@if ! systemctl --user is-active --quiet ros2-$(SERVICE_ROUTER).service 2>/dev/null; then \
		echo "ERROR: Zenoh router is not running. Start it first with: make start-zenoh-router"; \
		exit 1; \
	fi
	@echo "======================================================================"
	@echo "Starting stress test with Zenoh + Shared Memory"
	@echo "======================================================================"
	@echo "Config: $(STRESS_CONFIG) ($(PROFILE_NAME))"
	@echo "Resolution: $(WIDTH)x$(HEIGHT) @ $(FPS) FPS"
	@$(eval DATA_RATE_MB := $(shell echo "scale=1; $(WIDTH) * $(HEIGHT) * 3 * $(FPS) / 1048576" | bc 2>/dev/null || echo "N/A"))
	@echo "Estimated data rate: ~$(DATA_RATE_MB) MB/s (uncompressed)"
	@echo "QoS: $(QOS_RELIABILITY)/$(QOS_DURABILITY)"
	@echo "======================================================================"
	@mkdir -p $(GSCAM_STRESS_DIR)/zenoh
	@$(eval CSV_DATETIME := $(shell date +%Y-%m-%d_%H-%M-%S))
	@$(eval CSV_PATH := $(GSCAM_STRESS_DIR)/zenoh/$(CSV_DATETIME).csv)
	@echo "CSV output: $(CSV_PATH)"
	@. /opt/ros/humble/setup.sh && \
	. $(GSCAM_STRESS_DIR)/install/setup.sh && \
	ros2 systemd launch \
		--name $(SERVICE_ZENOH) \
		--replace \
		--domain-id $(DOMAIN_ID_ZENOH) \
		--rmw rmw_zenoh_cpp \
		--source $(GSCAM_STRESS_DIR)/install/setup.sh \
		--env ZENOH_CONFIG=file://$(RMW_CONFIG_DIR)/zenoh_shm.json5 \
		--description 'RMW Stress Test with Zenoh ($(PROFILE_NAME))' \
		rmw_stress_test stress_test.launch.py \
		width:=$(WIDTH) \
		height:=$(HEIGHT) \
		fps:=$(FPS) \
		csv_output_path:=$(CSV_PATH) \
		qos_reliability:=$(QOS_RELIABILITY) \
		qos_durability:=$(QOS_DURABILITY) \
		qos_history:=$(QOS_HISTORY) \
		qos_history_depth:=$(QOS_HISTORY_DEPTH) \
		qos_deadline_ms:=$(QOS_DEADLINE_MS) \
		qos_liveliness:=$(QOS_LIVELINESS) \
		qos_liveliness_lease_ms:=$(QOS_LIVELINESS_LEASE_MS)

.PHONY: stop-zenoh-test
stop-zenoh-test:
	@. /opt/ros/humble/setup.sh && ros2 systemd stop $(SERVICE_ZENOH)

.PHONY: status-zenoh-test
status-zenoh-test:
	@. /opt/ros/humble/setup.sh && ros2 systemd status $(SERVICE_ZENOH)

.PHONY: logs-zenoh-test
logs-zenoh-test:
	@. /opt/ros/humble/setup.sh && ros2 systemd logs $(SERVICE_ZENOH)

.PHONY: start-zenoh stop-zenoh status-zenoh logs-zenoh
start-zenoh: start-zenoh-test
stop-zenoh: stop-zenoh-test
status-zenoh: status-zenoh-test
logs-zenoh: logs-zenoh-test

.PHONY: stop-all
stop-all:
	@echo "Stopping all services..."
	@. /opt/ros/humble/setup.sh && \
	for service in $(SERVICE_CYCLONEDDS) $(SERVICE_ZENOH) $(SERVICE_ROUTER); do \
		if systemctl --user is-active --quiet ros2-$$service.service 2>/dev/null; then \
			echo "Stopping $$service"; \
			ros2 systemd stop $$service; \
		fi; \
	done
	@pkill -x iox-roudi && echo "Stopped Iceoryx RouDi" || true

.PHONY: status-all
status-all:
	@echo "Stress Test Services:"
	@if pgrep -x "iox-roudi" > /dev/null; then \
		echo "  Iceoryx RouDi: ACTIVE (PID: $$(pgrep -x iox-roudi))"; \
	else \
		echo "  Iceoryx RouDi: INACTIVE"; \
	fi
	@for service in $(SERVICE_CYCLONEDDS) $(SERVICE_ZENOH) $(SERVICE_ROUTER); do \
		if systemctl --user is-active --quiet ros2-$$service.service 2>/dev/null; then \
			echo "  $$service: ACTIVE"; \
		else \
			echo "  $$service: INACTIVE"; \
		fi; \
	done

# Benchmark automation targets
.PHONY: benchmark
benchmark:
	@echo "======================================================================"
	@echo "Running Complete Benchmark Suite"
	@echo "======================================================================"
	@echo "This will test 14 configurations with both CycloneDDS and Zenoh"
	@echo "Estimated duration: ~35 minutes"
	@echo ""
	@$(GSCAM_STRESS_DIR)/scripts/run_boundary_test.sh 60

.PHONY: benchmark-quick
benchmark-quick:
	@echo "======================================================================"
	@echo "Running Quick Benchmark"
	@echo "======================================================================"
	@echo "This will test 6 key configurations with both RMW implementations"
	@echo "Estimated duration: ~8 minutes"
	@echo ""
	@$(GSCAM_STRESS_DIR)/scripts/quick_boundary_test.sh

.PHONY: benchmark-analyze
benchmark-analyze:
	@echo "======================================================================"
	@echo "Analyzing Benchmark Results"
	@echo "======================================================================"
	@$(eval LATEST_RUN := $(shell ls -td $(GSCAM_STRESS_DIR)/results/run_* 2>/dev/null | head -1))
	@$(eval LATEST_QUICK := $(shell ls -td $(GSCAM_STRESS_DIR)/results/quick_run_* 2>/dev/null | head -1))
	@if [ -n "$(LATEST_RUN)" ]; then \
		echo "Analyzing full benchmark: $(LATEST_RUN)"; \
		$(GSCAM_STRESS_DIR)/scripts/analyze_results.py $(LATEST_RUN)/summary.csv --report $(LATEST_RUN)/findings.txt; \
	elif [ -n "$(LATEST_QUICK)" ]; then \
		echo "Analyzing quick benchmark: $(LATEST_QUICK)"; \
		$(GSCAM_STRESS_DIR)/scripts/analyze_results.py $(LATEST_QUICK)/summary.csv --report $(LATEST_QUICK)/findings.txt; \
	else \
		echo "No benchmark results found. Run 'make benchmark' or 'make benchmark-quick' first."; \
		exit 1; \
	fi
