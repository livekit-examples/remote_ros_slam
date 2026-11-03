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

import pytest

foxglove_layout = pytest.importorskip('livekit_scarf_slam.foxglove_layout')

TEMPLATE = '/scarf_slam/submaps/chunk_000'


def _layout(settings):
    return {
        'configById': {
            '3D!a': {'topics': {TEMPLATE: settings, '/tf': {'visible': True}}},
            'Image!b': {'imageTopic': '/stereo/left/image_rect'},
        },
        'layout': '3D!a',
    }


def test_expand_layout_copies_the_template_settings():
    settings = {'visible': True, 'pointSize': 3, 'colorMode': 'rgb'}
    layout = _layout(settings)
    assert foxglove_layout.expand_layout(layout, 4) == 1
    topics = layout['configById']['3D!a']['topics']
    assert sorted(topics) == ['/scarf_slam/submaps/chunk_000', '/scarf_slam/submaps/chunk_001',
                              '/scarf_slam/submaps/chunk_002', '/scarf_slam/submaps/chunk_003',
                              '/tf']
    assert all(topics[f'/scarf_slam/submaps/chunk_00{i}'] == settings for i in range(4))
    # The copies are independent, and other panels and topics do not change.
    topics['/scarf_slam/submaps/chunk_001']['pointSize'] = 9
    assert topics[TEMPLATE]['pointSize'] == 3
    assert topics['/tf'] == {'visible': True}
    assert layout['configById']['Image!b'] == {'imageTopic': '/stereo/left/image_rect'}


def test_expand_layout_overwrites_old_slot_settings():
    layout = _layout({'visible': True})
    layout['configById']['3D!a']['topics']['/scarf_slam/submaps/chunk_001'] = {'visible': False}
    foxglove_layout.expand_layout(layout, 2)
    assert layout['configById']['3D!a']['topics']['/scarf_slam/submaps/chunk_001'] == {
        'visible': True}


def test_expand_layout_requires_the_template():
    with pytest.raises(ValueError, match='chunk_000'):
        foxglove_layout.expand_layout({'configById': {'3D!a': {'topics': {}}}}, 4)
