# Henki ROS 2 Best Practices

> Source: https://github.com/henki-robotics/henki_ros2_best_practices
> By Henki Robotics. Included here as a reference for AI agents and team members.

## Nodes
- Build ROS 2 Nodes to have a single responsibility.
- Separate application logic into its own class or library, keeping ROS 2 nodes focused solely on communication. This improves code clarity, testability, and maintainability.

## Launch files
- Generate launch files in `launch` folder.
- Prefer XML launch format. Python launch files are not intended to be used as the launch front-end, but can be still used if needed for flexibility.
- Avoid passing or hardcoding node parameters in launch files; use config files instead.

## Parameters and Config Files
- Store ROS node parameters in YAML files under the `config` folder.  
- Treat package-level YAML files as "default" parameters. Users should not modify these files directly. Instead, they can override parameters in their own copies.
- If parameters are expected to be changed in runtime, make the parameters dynamic by defining parameter callbacks.

## Logging
- Use ROS 2 logger instead of `print()` and `std::cout` statements. 
- Use relevant log levels to avoid log noise:
  - `INFO`: System operates normally, logged as informational purposes.
  - `WARN`: Unexpected, but recoverable condition. Might require action.
  - `ERROR`: Critical failure. System no longer operates correctly. Requires immediate action.
- Avoid log spam. A usual mistake is to add logs on high-frequency callbacks, which flood the logs. For these cases, prefer throttled logs.

## Message Interfaces
- Aim to reuse existing message interfaces, for example from the [common_interfaces](https://github.com/ros2/common_interfaces/) repository. Examples:
    - `geometry_msgs`
    - `sensor_msgs`
    - `nav_msgs`
    - `tf2_msgs`
- If there isn't a viable existing message type that fits well for your use-case, create custom message interface.
- Custom message interfaces should be in their own packages with `_msgs` suffix. This allows easy message reusage without depending on the full package.
- Avoid using primitive type messages from the `std_msgs` or `example_interfaces` packages, such as `Float32`, `Bool` and `String`, as they are meant to be used only for quick prototyping and example usage. Instead, use a custom message with a semantic meaning.
  - This doesn't apply to general messages from `std_msgs`, such as `Header`
- Enum's in message interfaces can be simulated with constants. See for example `level` in [DiagnosticStatus](https://github.com/ros2/common_interfaces/blob/rolling/diagnostic_msgs/msg/DiagnosticStatus.msg) message.

## Actions and Services
- Use services only for very fast executing tasks (less than a second), such as requesting or setting a state. Use actions for tasks that take time to execute, can have multiple error cases or may require cancellation.
- Prefer to use enum-style error codes instead of string messages when an action can fail for multiple reasons. This allows clients to reliably parse errors and enables user-friendly messages, including support for multiple languages.

## Code Style
- Refer to official ROS 2 [code style guide](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).
- Follow the existing project coding style for all legacy code to maintain consistency. Apply updated best practices only to new nodes or when refactoring code.

## Performance
- Prefer C++ over Python for performance-critical nodes with high-frequency control loops and high-bandwidth data. Python works well for example for tooling, high-level orchestration, testing and prototyping.
- Use C++ composable nodes with intra-process communication enabled to pass large data such as images and point clouds between nodes. This avoids large memory and DDS overhead.

## Executors and Callback Groups
- Prefer `SingleThreadedExecutor` over `MultiThreadedExecutor` where possible. Single threaded applications are usually cleaner, easier to test, have deterministic execution order, and have a lower performance impact. 
  - It can be tempting to use `MultiThreadedExecutor` for any node to allow synchronous communication inside other callbacks. This can be often avoided with good Node design or asynchronous communication.
  - For applications that truly require multi-threading, `MultiThreadedExecutor` is a great option.

## Dependencies
- Always define ROS 2 package dependencies on package-level.
  - Use `package.xml` to automatically install dependencies using `rosdep`
  - If a package isn't available in `rosdep`, prefer to make a PR for rosdistro ([instructions](https://github.com/ros/rosdistro/blob/master/CONTRIBUTING.md#rosdep-rules-contributions)).
  - Rosdep doesn't support exact version tagging. If you require specific version for example of a Python package, use `requirements.txt` file.
- If your project requires a specific version of external dependencies to be built from source, `vcs_tool` ([link](https://github.com/dirk-thomas/vcstool)) is commonly used to define such dependencies in `.repos` file format.
  - Use an exact version of the repository, such as a commit hash or tag to avoid regressions.
- Do not rebuild external ROS 2 packages just to change configuration or launch files. Instead, make a copy of the launch and config files in your workspace and modify them there.

## Documentation
- Document each ROS package with `README.md` file that has the following information for each node:
  - Short description and overview
  - Usage
  - API: Topics, Services and Actions
  - Parameters with type, description and default value.

## Testing
- At minimum, write thorough unit tests for core application logic, using mocks where appropriate to isolate the tested functionality. If the application logic is well-separated from the ROS 2 nodes, the nodes themselves do not require unit tests.
- Test ROS 2 nodes, including their communication behavior, as part of integration testing.
- Aim for high test coverage (90-100%).
- Avoid using arbitrary sleeps in tests, as they can make tests non-deterministic and flaky. Instead of sleeping to wait for a ROS message, use synchronization mechanisms or wait for the expected result with a proper timeout.
