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
Paced sending of messages on many topics, for ROS Portal DataTracks.

Measured through LiveKit Cloud with ROS Portal v0.2.0 (one room, two portals):

- Messages published back to back on different topics are mostly lost: of a
  burst of 10 messages of 3 KB, 2 arrived, in every round. With 2 ms or more
  between the messages, about 1% were lost.
- The first message on a new topic is always lost. ROS Portal creates the
  DataTrack when that message arrives, before a viewer can subscribe to it.
- The rest are lost at random, and more of them the faster the bytes go out.
  3 KB messages 5 ms apart (4.8 Mbps): about 1% lost. 30 KB messages 20 ms
  apart (12 Mbps): 25%. The ScaRF-SLAM map at 5 cm voxels, in 6.4 KB chunks
  5 ms apart (10 Mbps): 20%.

So a background thread sends the messages at most max_rate_bytes_per_s, and
at least gap_s apart: after a message of n bytes, it waits
max(gap_s, n / max_rate_bytes_per_s). The latest message of a topic replaces
an unsent older one. A topic counts as new
for warmup_s after its first message, and the last message sent in that time
is sent again once the topic is warm.

The last message of every topic is also sent again on a timer, so that a lost
message or a viewer that joins late catches up when no new messages come, for
example after the mapping stops. Every refresh_interval_s, the topic sent
least recently is sent again, once its last message is refresh_interval_s old.
With the default of 1 s, that is at most one extra message per second, and
each topic of a 20-topic map is sent again every 20 s.

The module has no ROS dependency. The caller passes the send function.
"""

from collections import OrderedDict
import threading
import time


class PacedSender:
    """Send (topic, message) pairs from a thread, at a capped rate."""

    def __init__(self, send, gap_s=0.005, warmup_s=2.0, refresh_interval_s=1.0,
                 max_rate_bytes_per_s=0.0, size=len, clock=time.monotonic,
                 sleep=time.sleep):
        """
        Start the sender thread.

        size returns the byte count of a message, for the rate cap. A
        max_rate_bytes_per_s of 0 turns the cap off.
        """
        if gap_s < 0.0:
            raise ValueError('gap_s must not be negative')
        if max_rate_bytes_per_s < 0.0:
            raise ValueError('max_rate_bytes_per_s must not be negative (0 turns it off)')
        self._max_rate = max_rate_bytes_per_s
        self._size = size
        if warmup_s < 0.0:
            raise ValueError('warmup_s must not be negative')
        if refresh_interval_s < 0.0:
            raise ValueError('refresh_interval_s must not be negative (0 turns it off)')
        self._refresh_interval_s = refresh_interval_s
        self._sent_at = {}
        self._last_refresh = clock()
        self._send = send
        self._gap_s = gap_s
        self._warmup_s = warmup_s
        self._clock = clock
        self._sleep = sleep
        self._pending = OrderedDict()
        # Per topic: the time of its first message, the last message sent, and
        # when that message is due again because the topic was new.
        self._first_sent = {}
        self._last_sent = {}
        self._resend_at = {}
        self._closed = False
        self._condition = threading.Condition()
        self._thread = threading.Thread(target=self._run, name='paced_sender', daemon=True)
        self._thread.start()

    def put(self, topic, msg):
        """Queue msg on topic. It replaces an unsent message of the topic."""
        with self._condition:
            if self._closed:
                raise RuntimeError('the sender is closed')
            self._pending.pop(topic, None)
            self._pending[topic] = msg
            self._condition.notify()

    def pending(self):
        """Return the number of queued messages, without the resends."""
        with self._condition:
            return len(self._pending)

    def close(self, timeout=5.0):
        """Send the queued messages, skip the resends, and stop the thread."""
        with self._condition:
            self._closed = True
            self._condition.notify()
        self._thread.join(timeout)

    def _next(self):
        """Return the next (topic, msg) to send, or the seconds to wait for one."""
        if self._pending:
            return self._pending.popitem(last=False)
        if self._closed:
            return None
        now = self._clock()
        wait = 60.0
        if self._resend_at:
            topic = min(self._resend_at, key=self._resend_at.get)
            due = self._resend_at[topic] - now
            if due <= 0.0:
                del self._resend_at[topic]
                return topic, self._last_sent[topic]
            wait = due
        if self._refresh_interval_s and self._sent_at:
            topic = min(self._sent_at, key=self._sent_at.get)
            due = max(self._last_refresh, self._sent_at[topic]) + self._refresh_interval_s - now
            if due <= 0.0:
                self._last_refresh = now
                return topic, self._last_sent[topic]
            wait = min(wait, due)
        return wait

    def _run(self):
        while True:
            with self._condition:
                item = self._next()
                while isinstance(item, float):
                    self._condition.wait(item)
                    item = self._next()
            if item is None:
                return
            topic, msg = item
            self._send(topic, msg)
            now = self._clock()
            first = self._first_sent.setdefault(topic, now)
            with self._condition:
                self._last_sent[topic] = msg
                self._sent_at[topic] = now
                if now - first < self._warmup_s:
                    self._resend_at[topic] = first + self._warmup_s
                else:
                    self._resend_at.pop(topic, None)
            wait = self._gap_s
            if self._max_rate:
                wait = max(wait, self._size(msg) / self._max_rate)
            if wait:
                self._sleep(wait)
