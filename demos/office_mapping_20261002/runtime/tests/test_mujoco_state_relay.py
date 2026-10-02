import socket
import struct
import sys
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from go2w_mujoco_state_relay import MujocoStateRelay, decode_state


class StateRelayTests(unittest.TestCase):
    def test_invalid_packet_rejected(self):
        for packet in (b"", struct.pack("<dii", 0, 9999, 1), struct.pack("<dii", 0, 26, 25)):
            with self.assertRaises(ValueError):
                decode_state(packet)

    def test_forwards_exact_bytes_and_tracks_joint_reference(self):
        scene = ROOT / "matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_yard_extended_nav.xml"
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as ue, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            ue.bind(("127.0.0.1", 0))
            ue.settimeout(2)
            relay = MujocoStateRelay(scene, source_port=0, ue_port=ue.getsockname()[1])
            try:
                qpos = [0.] * 26
                qpos[3] = 1.
                qpos[23:] = [2.5, -2.2, 1.75]
                packet = (struct.pack("<di26d", 10., 26, *qpos)
                          + struct.pack("<i25d", 25, *([0.] * 25)) + struct.pack("<i", 0))
                sender.sendto(packet, relay.socket.getsockname())
                self.assertEqual(ue.recvfrom(65535)[0], packet)
                deadline = time.monotonic() + 2
                while relay.latest is None and time.monotonic() < deadline:
                    time.sleep(.01)
                positions = dict(relay.positions())
                self.assertEqual(positions["pedestrian_ramp_exit"], (2.5, 20.8, .87))
                self.assertEqual(positions["pedestrian_north_corridor"], (-2.2, 25.8, .87))
                self.assertEqual(positions["pedestrian_gallery_crossing"], (10.25, 34.25, .87))
                relay.latest = (time.monotonic() - 1., 10., tuple(qpos))
                with self.assertRaises(ValueError):
                    relay.positions()
            finally:
                relay.close()


if __name__ == "__main__":
    unittest.main()
