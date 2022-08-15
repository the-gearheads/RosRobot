#!/usr/bin/env bash

touch ./src/external/navigation2/nav2_system_tests/COLCON_IGNORE

colcon build --symlink-install $@ --cmake-args -DCMAKE_CXX_FLAGS:STRING=-Wno-error=maybe-uninitialized
