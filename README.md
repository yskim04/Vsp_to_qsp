# Repository Guidelines

## Project Structure & Module Organization
- `src/velocity_to_attitude_node.cpp`: main ROS2 node that converts PX4 velocity setpoints to attitude/thrust commands.
- `launch/offboard_velocity_to_attitude.launch.py`: launch entrypoint with a configurable `params_file` argument.
- `config/offboard_velocity_to_attitude.params.yaml`: default controller gains, limits, and PX4 topic mappings.
- `CMakeLists.txt` and `package.xml`: build, dependencies, and package metadata.
- `ROS2_Offboard_Velocity_to_Attitude_Design.md`: design reference; update it when control logic or assumptions change.

## Build, Test, and Development Commands
Run from ROS2 workspace root (the directory containing `src/`):

```bash
colcon build --packages-select vsp_to_qsp
```
Builds the C++17 node and installs launch/config assets.

```bash
colcon test --packages-select vsp_to_qsp
colcon test-result --verbose
```
Executes available tests/lint checks and prints detailed results.

```bash
source install/setup.bash
ros2 launch vsp_to_qsp offboard_velocity_to_attitude.launch.py
```
Starts the node with default parameters.

```bash
ros2 launch vsp_to_qsp offboard_velocity_to_attitude.launch.py \
  params_file:=/absolute/path/custom.params.yaml
```
Overrides the default parameter file.

If `px4_msgs` is not auto-detected, add `--cmake-args -DVSP_TO_QSP_PX4_MSGS_PREFIX=/path/to/install` during `colcon build`.

## Coding Style & Naming Conventions
- C++: C++17, 2-space indentation, braces on new lines.
- Naming: `PascalCase` for types/classes, `snake_case` for functions, variables, parameters, and topic keys.
- Python launch file: PEP 8 (4-space indentation, descriptive names).
- Keep controller constants as `constexpr` near the top and isolate math/saturation logic in helper functions.

## Testing Guidelines
- Add tests when changing controller behavior, saturation logic, timeout handling, or input validation.
- Prefer behavior-based test names such as `test_timeout_resets_command`.
- Before PRs, verify build, tests, and a local launch run.

## Commit & Pull Request Guidelines
- Current history uses short imperative lowercase subjects (for example, `fix build error`). Follow that style and keep subjects concise.
- PRs should include: purpose, behavior impact, parameter/topic changes, and validation steps.
- Attach logs, plots, or reproduction notes when output commands or flight behavior change.
