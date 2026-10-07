#!/usr/bin/env bash
set -eo pipefail
source /opt/nav_ws/install/setup.bash
cmake -S /opt/hmi -B /opt/hmi_build -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_TEST=OFF
cmake --build /opt/hmi_build --parallel 2
cmake -S /opt/hmi/tests/field -B /opt/hmi_field_tests
cmake --build /opt/hmi_field_tests --parallel 2
ctest --test-dir /opt/hmi_field_tests --output-on-failure
