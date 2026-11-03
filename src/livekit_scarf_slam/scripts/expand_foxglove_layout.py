#!/usr/bin/env python3

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
Copy the settings of one map chunk topic to every chunk slot of a Foxglove layout.

Configure /scarf_slam/submaps/chunk_000 in a 3D panel, export the layout
(Layouts > Export), run this script on the file, and import the result.
"""

import argparse
import json
from pathlib import Path
import sys

from livekit_scarf_slam.foxglove_layout import DEFAULT_TEMPLATE_TOPIC
from livekit_scarf_slam.foxglove_layout import expand_layout


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('layout', type=Path, help='exported Foxglove layout JSON')
    parser.add_argument(
        'output', type=Path, nargs='?', help='layout to write (default: the input layout)')
    parser.add_argument(
        '--count', type=int, default=256, help='number of chunk slots (default: 256)')
    parser.add_argument(
        '--template', default=DEFAULT_TEMPLATE_TOPIC,
        help=f'topic whose settings are copied (default: {DEFAULT_TEMPLATE_TOPIC})')
    args = parser.parse_args()

    layout = json.loads(args.layout.read_text())
    try:
        panels = expand_layout(layout, args.count, args.template)
    except ValueError as error:
        sys.exit(f'error: {error}')
    output = args.output or args.layout
    output.write_text(json.dumps(layout, indent=2) + '\n')
    print(f'Copied the settings of {args.template} to {args.count} chunk slots '
          f'in {panels} panel(s): {output}')


if __name__ == '__main__':
    main()
