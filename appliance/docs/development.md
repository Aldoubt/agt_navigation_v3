# Development

`Dockerfile.dev` extends the tested field image. Mount independent checkouts at /opt/nav_ws/src/agt_navigation_v3, /opt/mapping_ws/src/agt-lio-pgo-mapping, /opt/hmi and host ~/agt at /data. Persist workspace build/install directories in named volumes for incremental colcon and Qt CMake builds. Do not put mapping source inside navigation source or copy navigation into Qt.

Source changes to pure appliance Python can be tested with PYTHONPATH=appliance python3 -m pytest appliance/tests; format with ruff using appliance/pyproject.toml, then python3 -m build appliance. ROS2 callback/action/native tests must run in Humble, not be claimed from Python-only mocks. Qt logic test: cmake -S <hmi>/tests/field -B <build>; cmake --build <build>; ctest --test-dir <build> --output-on-failure. Full HMI builds with ROS2 channel enabled by ROS_VERSION=2 after sourcing Humble.

Incremental core builds: source /opt/ros/humble/setup.bash; cd /opt/nav_ws; colcon build --packages-select <changed package>; source install/setup.bash. Mapping is a separate workspace; never source both FAST-LIO/interface overlays into one mapping process. The controlled with_overlay.sh wrapper resets the process into the intended mapping overlay. A debugger/VS Code/Codex can attach to Dockerfile.dev without rebuilding native dependencies.

Regenerate appliance/third_party/qt-field.patch from the pinned Qt base with git diff --binary; update patch_sha256 in repos.lock.yaml and rerun patch-apply test. Preserve upstream LICENSE/notices and modification dates. No pushing to Qt upstream is assumed.

ROS2 adapter integration fixture: PYTHONPATH=appliance python3 -m pytest appliance/tests_ros -s inside the Humble navigation overlay. It runs a real NavigateToPose action server/client, localized/LOST messages, measured odom and a mock explicit gateway socket. It never publishes nonzero velocity, and never claims real Nav2 planning or BBS/GICP hardware validation.
