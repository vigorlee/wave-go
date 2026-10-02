"""Predict clear time windows for this scene's transverse moving obstacles."""
import math
import xml.etree.ElementTree as ET

class CrossingYield:
    def __init__(self, scene):
        self.crossings = {}
        self.previous = None
        self.waiting = set()
        for body in ET.parse(scene).getroot().findall('.//worldbody/body'):
            joint = body.find('joint')
            if joint is None or not joint.get('name','').startswith('pedestrian_'):
                continue
            axis = tuple(map(float,joint.get('axis').split()))
            lateral = max(range(2),key=lambda k:abs(axis[k]))
            normal = 1-lateral
            origin = tuple(map(float,body.get('pos').split()))
            center = origin[lateral]-axis[lateral]*float(joint.get('ref','0'))
            mass = float(body.find('geom').get('mass'))
            omega = math.sqrt(float(joint.get('stiffness'))/mass)
            self.crossings[joint.get('name').removesuffix('_slide')] = (lateral,normal,center,omega)

    def evaluate(self, sim_time, qpos, positions):
        current = dict(positions)
        old = self.previous
        self.previous = (sim_time,current)
        if old is None or not 0 < sim_time-old[0] < 1:
            return True, ['waiting_for_crossing_velocity']
        dt = sim_time-old[0]
        # All approved crossings are traversed in the positive normal direction.
        blocked=[]
        for name,(lateral,normal,center,omega) in self.crossings.items():
            pos = current[name]
            ahead = pos[normal]-qpos[normal]
            entry_distance = 2.50 if name in self.waiting else 2.20
            if not 0 <= ahead <= entry_distance:
                continue
            velocity = (pos[lateral]-old[1][name][lateral])/dt
            horizon = (ahead+1.05)/.18+.50
            for sample in range(int(math.ceil(horizon/.10))+1):
                t = sample*.10
                future = center+(pos[lateral]-center)*math.cos(omega*t)+(velocity/omega)*math.sin(omega*t)
                if math.hypot(future-qpos[lateral], ahead-.18*t) < 1.05:
                    blocked.append(name)
                    break
        self.waiting = set(blocked)
        return bool(blocked),blocked
