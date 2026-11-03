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

import threading
import time

from livekit_scarf_slam.paced_sender import PacedSender
import pytest


class _Recorder:
    def __init__(self, block=None):
        self.sent = []
        self.times = []
        self._block = block
        self._lock = threading.Lock()

    def __call__(self, topic, msg):
        if self._block is not None:
            self._block.wait()
        with self._lock:
            self.sent.append((topic, msg))
            self.times.append(time.monotonic())

    def wait_for(self, count, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self._lock:
                if len(self.sent) >= count:
                    return list(self.sent)
            time.sleep(0.005)
        raise AssertionError(f'sent {self.sent}, expected {count} messages')


def test_sends_in_order_with_the_gap():
    recorder = _Recorder()
    sender = PacedSender(recorder, gap_s=0.02, warmup_s=0.0, refresh_interval_s=0.0)
    for i in range(5):
        sender.put(f'/t{i}', i)
    assert recorder.wait_for(5) == [(f'/t{i}', i) for i in range(5)]
    sender.close()
    gaps = [b - a for a, b in zip(recorder.times, recorder.times[1:])]
    assert min(gaps) >= 0.018


def test_latest_message_replaces_an_unsent_one():
    gate = threading.Event()
    recorder = _Recorder(block=gate)
    sender = PacedSender(recorder, gap_s=0.0, warmup_s=0.0, refresh_interval_s=0.0)
    sender.put('/busy', 0)  # The thread blocks on this one.
    time.sleep(0.05)
    sender.put('/a', 1)
    sender.put('/b', 2)
    sender.put('/a', 3)
    assert sender.pending() == 2
    gate.set()
    assert recorder.wait_for(3) == [('/busy', 0), ('/b', 2), ('/a', 3)]
    sender.close()


def test_resends_the_last_message_of_a_new_topic_once():
    recorder = _Recorder()
    sender = PacedSender(recorder, gap_s=0.0, warmup_s=0.2, refresh_interval_s=0.0)
    sender.put('/a', 1)
    recorder.wait_for(1)
    sender.put('/a', 2)
    assert recorder.wait_for(3) == [('/a', 1), ('/a', 2), ('/a', 2)]
    assert recorder.times[2] - recorder.times[0] >= 0.19
    time.sleep(0.3)
    assert len(recorder.sent) == 3
    # The topic is warm now, so a new message goes out once.
    sender.put('/a', 4)
    recorder.wait_for(4)
    time.sleep(0.3)
    assert recorder.sent[3:] == [('/a', 4)]
    sender.close()


def test_close_sends_the_queue_and_skips_resends():
    gate = threading.Event()
    recorder = _Recorder(block=gate)
    sender = PacedSender(recorder, gap_s=0.0, warmup_s=10.0, refresh_interval_s=10.0)
    sender.put('/a', 1)
    time.sleep(0.05)
    sender.put('/b', 2)
    gate.set()
    sender.close()
    assert recorder.sent == [('/a', 1), ('/b', 2)]
    with pytest.raises(RuntimeError):
        sender.put('/c', 3)


def test_refresh_resends_the_least_recently_sent_topic_on_the_timer():
    recorder = _Recorder()
    sender = PacedSender(recorder, gap_s=0.0, warmup_s=0.0, refresh_interval_s=0.2)
    sender.put('/a', 1)
    sender.put('/b', 2)
    recorder.wait_for(2)
    sender.put('/a', 3)
    recorder.wait_for(3)
    # /b is the oldest, then /a with its latest message, one per interval.
    sent = recorder.wait_for(5, timeout=2.0)
    assert sent[3:5] == [('/b', 2), ('/a', 3)]
    gaps = [b - a for a, b in zip(recorder.times[2:], recorder.times[3:5])]
    assert min(gaps) >= 0.19
    sender.close()


def test_refresh_never_resends_a_topic_sooner_than_the_interval():
    recorder = _Recorder()
    sender = PacedSender(recorder, gap_s=0.0, warmup_s=0.0, refresh_interval_s=0.3)
    sender.put('/a', 1)
    recorder.wait_for(1)
    time.sleep(0.2)
    assert len(recorder.sent) == 1
    recorder.wait_for(2, timeout=1.0)
    assert recorder.times[1] - recorder.times[0] >= 0.29
    sender.close()


def test_rate_cap_spaces_messages_by_their_size():
    recorder = _Recorder()
    sender = PacedSender(recorder, gap_s=0.001, warmup_s=0.0, refresh_interval_s=0.0,
                         max_rate_bytes_per_s=10_000.0)
    sender.put('/big', b'x' * 2000)  # 0.2 s at 10 KB/s.
    sender.put('/small', b'x')  # The gap: 0.001 s.
    sender.put('/last', b'x')
    recorder.wait_for(3)
    sender.close()
    assert recorder.times[1] - recorder.times[0] >= 0.19
    assert recorder.times[2] - recorder.times[1] < 0.1


@pytest.mark.parametrize('kwargs', [
    {'gap_s': -1.0}, {'warmup_s': -1.0}, {'refresh_interval_s': -1.0},
    {'max_rate_bytes_per_s': -1.0}])
def test_rejects_negative_times(kwargs):
    with pytest.raises(ValueError):
        PacedSender(lambda *_: None, **kwargs)
