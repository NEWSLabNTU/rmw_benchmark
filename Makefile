# Top-level Makefile for RMW Benchmark
# Provides dependency installation and delegates to subdirectory Makefiles

.PHONY: help
help:
	@echo "RMW Benchmark - Top-level Makefile"
	@echo ""
	@echo "======================================================================"
	@echo "Installation"
	@echo "======================================================================"
	@echo "  install-deps         Install all required system dependencies"
	@echo "  install-ros-deps     Install ROS 2 packages only"
	@echo "  install-python-deps  Install Python packages only"
	@echo "  verify-deps          Verify all dependencies are installed"
	@echo ""
	@echo "======================================================================"
	@echo "Running Tests"
	@echo "======================================================================"
	@echo ""
	@echo "To run benchmarks or tests, navigate to the subdirectories:"
	@echo ""
	@echo "  cd gscam_stress                      # Stress test benchmarks"
	@echo "  cd autoware_planning_simulation      # Planning simulation"
	@echo ""
	@echo "Then run 'make help' to see available targets."
	@echo ""
	@echo "See README.md for detailed usage instructions."

# =============================================================================
# Installation targets
# =============================================================================

.PHONY: install-deps
install-deps: install-ros-deps install-python-deps
	@echo ""
	@echo "======================================================================"
	@echo "All dependencies installed successfully!"
	@echo "======================================================================"
	@echo "Run 'make verify-deps' to verify installation"

.PHONY: install-ros-deps
install-ros-deps:
	@echo "======================================================================"
	@echo "Installing ROS 2 Humble dependencies..."
	@echo "======================================================================"
	@echo ""
	@echo "Installing RMW implementations..."
	sudo apt update
	sudo apt install -y ros-humble-rmw-cyclonedds-cpp ros-humble-rmw-zenoh-cpp
	@echo ""
	@echo "Installing Iceoryx for CycloneDDS shared memory support..."
	sudo apt install -y ros-humble-iceoryx-posh ros-humble-iceoryx-binding-c
	@echo ""
	@echo "Installing build tools..."
	sudo apt install -y python3-colcon-common-extensions
	@echo ""
	@echo "ROS 2 dependencies installed successfully!"

.PHONY: install-python-deps
install-python-deps:
	@echo "======================================================================"
	@echo "Installing Python dependencies..."
	@echo "======================================================================"
	@echo ""
	@echo "Installing ros2systemd for systemd service management..."
	pip3 install --user ros2systemd
	@echo ""
	@echo "Python dependencies installed successfully!"

.PHONY: verify-deps
verify-deps:
	@echo "======================================================================"
	@echo "Verifying dependencies..."
	@echo "======================================================================"
	@echo ""
	@echo "Checking ROS 2 packages..."
	@. /opt/ros/humble/setup.sh && \
	if ros2 pkg list | grep -q rmw_cyclonedds_cpp; then \
		echo "  ✓ rmw_cyclonedds_cpp installed"; \
	else \
		echo "  ✗ rmw_cyclonedds_cpp NOT FOUND"; \
		exit 1; \
	fi
	@. /opt/ros/humble/setup.sh && \
	if ros2 pkg list | grep -q rmw_zenoh_cpp; then \
		echo "  ✓ rmw_zenoh_cpp installed"; \
	else \
		echo "  ✗ rmw_zenoh_cpp NOT FOUND"; \
		exit 1; \
	fi
	@echo ""
	@echo "Checking Iceoryx packages..."
	@if dpkg -l | grep -q ros-humble-iceoryx-posh; then \
		echo "  ✓ iceoryx-posh installed"; \
	else \
		echo "  ✗ iceoryx-posh NOT FOUND"; \
		exit 1; \
	fi
	@if dpkg -l | grep -q ros-humble-iceoryx-binding-c; then \
		echo "  ✓ iceoryx-binding-c installed"; \
	else \
		echo "  ✗ iceoryx-binding-c NOT FOUND"; \
		exit 1; \
	fi
	@echo ""
	@echo "Checking Python packages..."
	@. /opt/ros/humble/setup.sh && \
	if ros2 systemd --help > /dev/null 2>&1; then \
		echo "  ✓ ros2systemd installed"; \
	else \
		echo "  ✗ ros2systemd NOT FOUND"; \
		echo "    Install with: pip3 install --user ros2systemd"; \
		exit 1; \
	fi
	@echo ""
	@echo "Checking build tools..."
	@if which colcon > /dev/null 2>&1; then \
		echo "  ✓ colcon installed"; \
	else \
		echo "  ✗ colcon NOT FOUND"; \
		exit 1; \
	fi
	@echo ""
	@echo "======================================================================"
	@echo "All dependencies verified successfully!"
	@echo "======================================================================"

