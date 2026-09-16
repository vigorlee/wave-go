#! /bin/bash

set -e

JSZR_CURRENT_PREFIX=$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

bash ${JSZR_CURRENT_PREFIX}/gazebo_dep.sh
bash ${JSZR_CURRENT_PREFIX}/gamepad_dep.sh
bash ${JSZR_CURRENT_PREFIX}/ros2_dep.sh
