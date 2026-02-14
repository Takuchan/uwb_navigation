# Changelog

## [Unreleased] - 2025-11-04

### Added
- **Dynamic anchor count support**: The system now supports any number of anchors from 3 to 26 (previously hardcoded to 3)
  - Number of anchors can be configured in `anchors.yaml` or via launch parameters
  - Anchor labels are automatically generated (a, b, c, d, ..., z)
  
- **Height parameters for 3D positioning**:
  - Added `anchor_height` parameter to specify the height of anchors from the ground
  - Added `tag_height` parameter to specify the height of the UWB tag on the robot
  - Distance calculations now account for height differences using 3D Euclidean distance
  - Parameters can be set in `anchors.yaml` or passed as launch arguments
  
- **Enhanced GUI for anchor placement**:
  - Added +/- buttons to adjust the number of anchors dynamically during placement
  - GUI displays current anchor and tag height settings
  - Enforces minimum (3) and maximum (26) anchor limits
  
- **Comprehensive documentation**:
  - `README_ANCHOR_CONFIG.md`: Configuration guide for UWB anchors
  - `USAGE_EXAMPLES.md`: Detailed usage examples and troubleshooting
  - Inline documentation in code

- **Test suite**: Added `test_anchor_config.py` with 7 test cases covering:
  - Backward compatibility with 3-anchor configuration
  - Multi-anchor configurations (5, 10 anchors)
  - Height parameter handling
  - YAML configuration parsing

### Changed
- **`uwb_ekf_node.py`**:
  - Refactored to dynamically load anchor positions based on `num_anchors` parameter
  - Added validation for anchor count (3-26)
  - Added helper methods `_convert_to_3d_position()` and `_get_height_difference()`
  - Updated `h_uwb()` and `H_uwb()` to use 3D distance calculations
  
- **`rviz_anchor_place.py`**:
  - Enhanced GUI with anchor count adjustment controls
  - Added height parameter display
  - Saves height parameters to YAML file
  - Added validation for anchor count limits
  
- **Launch files**:
  - `ekf_with_serial.launch.py`: Now reads height parameters from YAML and accepts launch arguments
  - `rviz_anchor_place.launch.py`: Added parameters for initial anchor count and heights
  - `uwb_nav.launch.py`: Added launch arguments for `anchor_height` and `tag_height` with forwarding to ekf_with_serial.launch.py

### Fixed
- Removed `__pycache__` files from git tracking
- Added `.gitignore` to prevent future Python cache file commits

### Compatibility
- Full backward compatibility maintained with existing 3-anchor configurations
- Existing `anchors.yaml` files without height parameters will use default values (0.0)
- All existing functionality preserved while adding new features

## Implementation Details

### Code Quality Improvements
- Extracted duplicate code into helper methods
- Added module-level constants for anchor limits
- Improved validation and error handling
- Enhanced code documentation

### Testing
All 7 unit tests pass successfully:
- Backward compatibility with 3 anchors
- Multi-anchor support (5+ anchors)
- Height parameter calculations
- YAML configuration parsing
- Individual z-coordinate support per anchor

### Usage Examples

#### Basic usage (backward compatible):
```bash
ros2 launch tk_uwb_ekf uwb_nav.launch.py
```

#### With 5 anchors and height parameters:
```bash
ros2 launch tk_uwb_ekf rviz_anchor_place.launch.py \
  num_anchors:=5 anchor_height:=1.5 tag_height:=0.3
  
# After placement and saving:
ros2 launch tk_uwb_ekf uwb_nav.launch.py \
  anchor_height:=1.5 tag_height:=0.3
```

#### With 10 anchors:
```bash
ros2 launch tk_uwb_ekf rviz_anchor_place.launch.py num_anchors:=10
# Then use GUI +/- buttons to adjust if needed
```

## Migration Guide

### For existing users:
No changes required! Your existing configuration will continue to work. To take advantage of new features:

1. **To use more anchors**: Use the anchor placement tool with `num_anchors` parameter
2. **To use height parameters**: Add them to your `anchors.yaml` or pass as launch arguments
3. **Rebuild after changes**: Run `colcon build --packages-select tk_uwb_ekf` after modifying `anchors.yaml`

### YAML format changes:
Old format (still supported):
```yaml
anchors:
- name: A0
  x: 0.0
  y: 0.0
```

New format with optional height parameters:
```yaml
anchors:
- name: A0
  x: 0.0
  y: 0.0
anchor_height: 1.5
tag_height: 0.3
```
