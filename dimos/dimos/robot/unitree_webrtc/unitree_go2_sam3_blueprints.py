#!/usr/bin/env python3

# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unitree Go2 SAM3 + exploration blueprint (no agent, no VLM/LLM).

This blueprint runs:
- Go2 WebRTC connection
- Voxel mapping + costmap
- Replanning A* navigation
- Wavefront frontier exploration (manual start via UI)
- SAM3 prompt search + web UI (http://localhost:7788 by default)
"""

from dimos.core.blueprints import autoconnect
from dimos.core.transport import JpegShmTransport
from dimos.mapping.costmapper import cost_mapper
from dimos.mapping.voxels import voxel_mapper
from dimos.msgs.sensor_msgs import Image
from dimos.navigation.frontier_exploration import wavefront_frontier_explorer
from dimos.navigation.replanning_a_star.module import replanning_a_star_planner
from dimos.robot.unitree.connection.go2 import go2_connection
from dimos.robot.unitree_webrtc.sam3_exploration import sam3_exploration


sam3_explore = autoconnect(
    go2_connection(),
    voxel_mapper(voxel_size=0.1),
    cost_mapper(),
    replanning_a_star_planner(),
    wavefront_frontier_explorer(),
    sam3_exploration(),
).global_config(robot_model="unitree_go2").transports(
    {
        # Raw Image over LCM can drop (too large). Use JPEG over shared memory for reliability.
        ("color_image", Image): JpegShmTransport("/color_image", quality=80),
    }
)


__all__ = ["sam3_explore"]
