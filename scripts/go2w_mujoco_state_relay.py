"""Relay the simulator's unmodified UE state and read actual pedestrian joints."""
from __future__ import annotations

import math
from pathlib import Path
import socket
import struct
import threading
import time
import xml.etree.ElementTree as ET


def decode_state(packet: bytes) -> tuple[float, tuple[float, ...]]:
    if len(packet) < 16:
        raise ValueError("short simulator state")
    sim_time, nq = struct.unpack_from("<di", packet)
    if not 0 < nq < 1024 or len(packet) < 16 + 8 * nq:
        raise ValueError(f"invalid state position count: {nq}")
    nv = struct.unpack_from("<i", packet, 12 + 8 * nq)[0]
    if not 0 < nv < 1024:
        raise ValueError(f"invalid state velocity count: {nv}")
    control_offset = 16 + 8 * (nq + nv)
    if len(packet) < control_offset + 4:
        raise ValueError("truncated simulator controls")
    nu = struct.unpack_from("<i", packet, control_offset)[0]
    if not 0 <= nu < 1024 or len(packet) != control_offset + 4 + 8 * nu:
        raise ValueError(f"unexpected state length: {len(packet)} for {nq}/{nv}/{nu}")
    qpos = struct.unpack_from(f"<{nq}d", packet, 12)
    if not all(math.isfinite(x) for x in (sim_time, *qpos)):
        raise ValueError("non-finite simulator state")
    return sim_time, qpos


def pedestrian_joints(scene: Path) -> tuple[int, list[tuple[str, int, tuple[float, ...], tuple[float, ...], float]]]:
    # MuJoCo merges include worldbodies before this scene's root bodies.
    roots: list[ET.Element] = []
    def expand(path: Path) -> None:
        root = ET.parse(path).getroot()
        for include in root.findall("include"):
            expand(path.parent / include.attrib["file"])
        roots.append(root)
    expand(scene)
    offset = 0
    actors = []
    def visit(body: ET.Element) -> None:
        nonlocal offset
        for node in body:
            if node.tag == "freejoint":
                offset += 7
            elif node.tag == "joint":
                if body.get("name", "").startswith("pedestrian_"):
                    if node.get("type") != "slide" or "quat" in body.attrib or "euler" in body.attrib:
                        raise ValueError("pedestrian relay requires an axis-aligned root slide")
                    actors.append((node.attrib["name"].removesuffix("_slide"), offset,
                                   tuple(map(float, body.attrib["pos"].split())),
                                   tuple(map(float, node.attrib["axis"].split())),
                                   float(node.get("ref", "0"))))
                offset += {"free": 7, "ball": 4}.get(node.get("type", "hinge"), 1)
            elif node.tag == "body":
                visit(node)
    for root in roots:
        world = root.find("worldbody")
        if world is not None:
            visit(world)
    return offset, actors


class MujocoStateRelay:
    def __init__(self, scene: Path, source_port: int = 25011, ue_port: int | None = None):
        self.nq, self.actors = pedestrian_joints(scene)
        if len(self.actors) != 3:
            raise ValueError("expected three physical pedestrian slide joints")
        self.destination = ("127.0.0.1", ue_port) if ue_port else None
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", source_port))
        self.socket.settimeout(0.2)
        self.latest: tuple[float, float, tuple[float, ...]] | None = None
        self.error: str | None = None
        self.packets = 0
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, name="mujoco-state-relay", daemon=True)
        self.thread.start()

    def run(self) -> None:
        while not self.stop.is_set():
            try:
                packet, _ = self.socket.recvfrom(65535)
                if self.destination is not None:
                    self.socket.sendto(packet, self.destination)
                self.packets += 1
                sim_time, qpos = decode_state(packet)
                if len(qpos) != self.nq:
                    raise ValueError(f"scene/state nq mismatch: {self.nq}/{len(qpos)}")
                self.latest = (time.monotonic(), sim_time, qpos)
                self.error = None
            except socket.timeout:
                pass
            except ValueError as exc:
                self.error = str(exc)
            except OSError as exc:
                if not self.stop.is_set():
                    self.error = str(exc)
                break

    def positions(self) -> tuple[tuple[str, tuple[float, float, float]], ...]:
        latest = self.latest
        if latest is None or time.monotonic() - latest[0] > 0.5:
            raise ValueError(self.error or "actual pedestrian state is unavailable or stale")
        qpos = latest[2]
        return tuple((name, tuple(p[k] + axis[k] * (qpos[index] - ref) for k in range(3)))
                     for name, index, p, axis, ref in self.actors)

    def close(self) -> None:
        self.stop.set()
        self.thread.join(timeout=0.5)
        self.socket.close()
