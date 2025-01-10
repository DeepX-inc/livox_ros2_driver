# CHANGELOG

## [Unreleased]

## [2025-01-11] - Toni Perälä

### Fixed
- Resolved a bug in the device broadcast address printing during initialization. Previously, the system printed a pointer instead of the actual device address. The array containing device broadcast addresses has been converted to a concatenated string that lists all whitelisted devices correctly.
- Updated device IDs in all configuration files in the Livox workspace. 
  - **Note**: For compatibility with `lidar_data_collection`, both `.json` and `.yaml` config files must include the device broadcast address.

### Changed
- Updated the package name in `CMakeLists.txt` and `package.xml`:
  - From: `deepx_livox_ros2_driver`
  - To: `livox_ros2_driver`
  - **Reason**: This change ensures consistency, as all Livox ROS2 driver-related nodes called from launch files are named accordingly.
- Set the current `.json` configuration for lidar launch to a test device.
  - **Note**: A shell script feature is under development to automatically configure the device ID using `tcpdump` data during the build process in `lidar_data_collection`. For standalone use, ensure the correct device broadcast address is added manually.

---
