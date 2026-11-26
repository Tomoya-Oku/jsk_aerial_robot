#!/usr/bin/env python3
"""Process-wide ownership of haptic actuator topics on the robot PC."""
import hashlib
import os
from pathlib import Path
import tempfile


class OutputLock:
    def __init__(self, topics, master_uri):
        self.files = []
        # Lock each topic separately: sharing either current or torque-enable
        # output is a conflict, even if the other topic has been remapped.
        try:
            for topic in sorted(set(topics)):
                key = hashlib.sha256(
                    (master_uri.rstrip('/') + '\n' + topic).encode()).hexdigest()
                path = Path(tempfile.gettempdir()) / ('dracomancer-' + key + '.lock')
                stream = path.open('a+b')
                self.files.append(stream)
                if os.name == 'nt':
                    import msvcrt
                    if stream.seek(0, 2) == 0:
                        stream.write(b'\0')
                        stream.flush()
                    stream.seek(0)
                    msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
                else:
                    import fcntl
                    fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as error:
            self.close()
            raise RuntimeError('Haptic output already owned or unavailable: ' + topic) from error

    def close(self):
        for stream in self.files:
            stream.close()
        self.files.clear()
        # Never unlink a lock file: another process may already hold that inode.


def claim_output(current_topic, torque_topic, *extra_topics):
    import rospy
    return OutputLock(
        [rospy.resolve_name(topic) for topic in (current_topic, torque_topic, *extra_topics)],
        os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'))
