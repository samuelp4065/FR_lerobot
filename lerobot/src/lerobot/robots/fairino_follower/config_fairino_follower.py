#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
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

from dataclasses import dataclass

from ..config import RobotConfig


@dataclass(kw_only=True)
class FairinoFollowerConfig(RobotConfig):
    """Configuration for the Fairino follower robot."""
    # IP address of the Fairino controller; this field is required and therefore
    # deliberately placed before fields with defaults. The previous version
    # declared a default value which triggered a ``TypeError`` during the
    # dataclass initialization because non-default arguments were defined after
    # defaulted ones in the parent class. Making ``controller_ip`` a required
    # keyword-only argument resolves the issue and ensures users explicitly
    # specify the target controller.
    controller_ip: str

    # Motion parameters for joint moves expressed in degrees per second.
    velocity: float = 30.0
    acceleration: float = 30.0