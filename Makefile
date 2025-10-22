# Top-level Makefile for Autoware RMW Zenoh Benchmark
# Delegates to subdirectory Makefiles

.PHONY: help
help:
	@echo "Autoware RMW Zenoh Benchmark - Top-level Makefile"
	@echo ""
	@echo "This Makefile delegates to subdirectories:"
	@echo "  - autoware_planning_simulation/"
	@echo "  - gscam_stress/"
	@echo ""
	@echo "Note: rmw_zenoh_cpp is installed as a system package (ros-humble-rmw-zenoh-cpp)"
	@echo ""
	@echo "====================================================================="
	@echo "Autoware Planning Simulation (autoware_planning_simulation/)"
	@echo "====================================================================="
	@echo "CycloneDDS targets:"
	@echo "  sim-start-cyclonedds      Start Autoware with CycloneDDS"
	@echo "  sim-stop-cyclonedds       Stop CycloneDDS simulation"
	@echo "  sim-status-cyclonedds     Show CycloneDDS service status"
	@echo "  sim-logs-cyclonedds       View CycloneDDS logs"
	@echo ""
	@echo "Zenoh targets:"
	@echo "  sim-start-router          Start Zenoh router daemon"
	@echo "  sim-stop-router           Stop Zenoh router"
	@echo "  sim-status-router         Show router status"
	@echo "  sim-logs-router           View router logs"
	@echo "  sim-start-zenoh           Start Autoware with Zenoh"
	@echo "  sim-stop-zenoh            Stop Zenoh simulation"
	@echo "  sim-status-zenoh          Show Zenoh service status"
	@echo "  sim-logs-zenoh            View Zenoh logs"
	@echo ""
	@echo "Management:"
	@echo "  sim-stop-all              Stop all simulation services"
	@echo "  sim-status-all            Show all simulation service statuses"
	@echo ""
	@echo "====================================================================="
	@echo "gscam Stress Test (gscam_stress/)"
	@echo "====================================================================="
	@echo "Build:"
	@echo "  stress-build              Build stress test package"
	@echo "  stress-clean              Clean build artifacts"
	@echo ""
	@echo "CycloneDDS:"
	@echo "  stress-start-cyclonedds   Start stress test with CycloneDDS"
	@echo "  stress-stop-cyclonedds    Stop CycloneDDS stress test"
	@echo "  stress-status-cyclonedds  Show CycloneDDS status"
	@echo "  stress-logs-cyclonedds    View CycloneDDS logs"
	@echo ""
	@echo "Zenoh:"
	@echo "  stress-start-router       Start Zenoh router for stress test"
	@echo "  stress-stop-router        Stop stress test router"
	@echo "  stress-status-router      Show router status"
	@echo "  stress-logs-router        View router logs"
	@echo "  stress-start-zenoh        Start stress test with Zenoh"
	@echo "  stress-stop-zenoh         Stop Zenoh stress test"
	@echo "  stress-status-zenoh       Show Zenoh status"
	@echo "  stress-logs-zenoh         View Zenoh logs"
	@echo ""
	@echo "Management:"
	@echo "  stress-stop-all           Stop all stress test services"
	@echo "  stress-status-all         Show all stress test service statuses"
	@echo ""
	@echo "Configuration:"
	@echo "  stress-show-config        Show stress test configuration"
	@echo "  stress-edit-config        Edit stress test configuration"
	@echo ""
	@echo "====================================================================="
	@echo "Global Management"
	@echo "====================================================================="
	@echo "  stop-all                  Stop ALL services (sim + stress)"
	@echo "  status-all                Show ALL service statuses"

# =============================================================================
# Autoware Planning Simulation targets
# =============================================================================

.PHONY: sim-start-cyclonedds sim-stop-cyclonedds sim-status-cyclonedds sim-logs-cyclonedds
sim-start-cyclonedds:
	$(MAKE) -C autoware_planning_simulation start-cyclonedds

sim-stop-cyclonedds:
	$(MAKE) -C autoware_planning_simulation stop-cyclonedds

sim-status-cyclonedds:
	$(MAKE) -C autoware_planning_simulation status-cyclonedds

sim-logs-cyclonedds:
	$(MAKE) -C autoware_planning_simulation logs-cyclonedds

.PHONY: sim-start-router sim-stop-router sim-status-router sim-logs-router
sim-start-router:
	$(MAKE) -C autoware_planning_simulation start-router

sim-stop-router:
	$(MAKE) -C autoware_planning_simulation stop-router

sim-status-router:
	$(MAKE) -C autoware_planning_simulation status-router

sim-logs-router:
	$(MAKE) -C autoware_planning_simulation logs-router

.PHONY: sim-start-zenoh sim-stop-zenoh sim-status-zenoh sim-logs-zenoh
sim-start-zenoh:
	$(MAKE) -C autoware_planning_simulation start-zenoh

sim-stop-zenoh:
	$(MAKE) -C autoware_planning_simulation stop-zenoh

sim-status-zenoh:
	$(MAKE) -C autoware_planning_simulation status-zenoh

sim-logs-zenoh:
	$(MAKE) -C autoware_planning_simulation logs-zenoh

.PHONY: sim-stop-all sim-status-all
sim-stop-all:
	$(MAKE) -C autoware_planning_simulation stop-all

sim-status-all:
	$(MAKE) -C autoware_planning_simulation status-all

# =============================================================================
# gscam Stress Test targets
# =============================================================================

.PHONY: stress-build stress-clean
stress-build:
	$(MAKE) -C gscam_stress build

stress-clean:
	$(MAKE) -C gscam_stress clean

.PHONY: stress-start-cyclonedds stress-stop-cyclonedds stress-status-cyclonedds stress-logs-cyclonedds
stress-start-cyclonedds:
	$(MAKE) -C gscam_stress start-cyclonedds

stress-stop-cyclonedds:
	$(MAKE) -C gscam_stress stop-cyclonedds

stress-status-cyclonedds:
	$(MAKE) -C gscam_stress status-cyclonedds

stress-logs-cyclonedds:
	$(MAKE) -C gscam_stress logs-cyclonedds

.PHONY: stress-start-router stress-stop-router stress-status-router stress-logs-router
stress-start-router:
	$(MAKE) -C gscam_stress start-router

stress-stop-router:
	$(MAKE) -C gscam_stress stop-router

stress-status-router:
	$(MAKE) -C gscam_stress status-router

stress-logs-router:
	$(MAKE) -C gscam_stress logs-router

.PHONY: stress-start-zenoh stress-stop-zenoh stress-status-zenoh stress-logs-zenoh
stress-start-zenoh:
	$(MAKE) -C gscam_stress start-zenoh

stress-stop-zenoh:
	$(MAKE) -C gscam_stress stop-zenoh

stress-status-zenoh:
	$(MAKE) -C gscam_stress status-zenoh

stress-logs-zenoh:
	$(MAKE) -C gscam_stress logs-zenoh

.PHONY: stress-stop-all stress-status-all
stress-stop-all:
	$(MAKE) -C gscam_stress stop-all

stress-status-all:
	$(MAKE) -C gscam_stress status-all

.PHONY: stress-show-config stress-edit-config
stress-show-config:
	$(MAKE) -C gscam_stress show-config

stress-edit-config:
	$(MAKE) -C gscam_stress edit-config

# =============================================================================
# Global Management
# =============================================================================

.PHONY: stop-all
stop-all:
	@echo "Stopping all services..."
	$(MAKE) -C autoware_planning_simulation stop-all
	$(MAKE) -C gscam_stress stop-all

.PHONY: status-all
status-all:
	@echo "====================================================================="
	@echo "Planning Simulation Services"
	@echo "====================================================================="
	$(MAKE) -C autoware_planning_simulation status-all
	@echo ""
	@echo "====================================================================="
	@echo "Stress Test Services"
	@echo "====================================================================="
	$(MAKE) -C gscam_stress status-all
