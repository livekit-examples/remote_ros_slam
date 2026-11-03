# Copyright 2026 LiveKit
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

"""
Copy the Foxglove settings of one map chunk topic to every chunk slot.

Foxglove keeps panel settings per topic name. Configure one chunk topic in the
3D panel, export the layout, and copy its settings to the slots, so that every
chunk of a run shows the same way. The settings are copied as they are, so the
layout format does not matter: every object in the layout that has the
template topic as a key gets the slot topics too.
"""

import copy

from livekit_scarf_slam.submap_cloud import chunk_topic

DEFAULT_TEMPLATE_TOPIC = chunk_topic(0)


def expand_layout(layout, count, template_topic=DEFAULT_TEMPLATE_TOPIC):
    """
    Give chunk slots 0 to count - 1 the settings of template_topic, in place.

    Return the number of topic maps that had the template topic. Raise
    ValueError if none had it.
    """
    slots = [chunk_topic(slot) for slot in range(count)]
    found = 0

    def visit(node):
        nonlocal found
        if isinstance(node, dict):
            template = node.get(template_topic)
            if isinstance(template, dict):
                found += 1
                for topic in slots:
                    if topic != template_topic:
                        node[topic] = copy.deepcopy(template)
            for value in list(node.values()):
                visit(value)
        elif isinstance(node, list):
            for value in node:
                visit(value)

    visit(layout)
    if not found:
        raise ValueError(
            f'the layout has no settings for {template_topic}. Show that topic in a 3D '
            'panel, change one of its settings, and export the layout again.')
    return found
