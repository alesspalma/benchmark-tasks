"""Cut resistance, intact tissue bond and frictional core retention."""

import numpy as np

from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


def limited(v, bound):
    return v * min(1.0, bound / max(np.linalg.norm(v), 1.e-9))


class Tissue:
    def __init__(self):
        self.reset()

    def reset(self):
        self.cut = 0.
        self.work = 0.
        self.peak = 0.
        self.separated = False
        self.engagement = None
        self.error = None

    def advance(self, tip, tool_v, fruit, fruit_v, core, core_v):
        r_fruit = quat_xyzw_to_matrix(fruit[3:])
        vertical = r_fruit[:, 2]
        shaft = quat_xyzw_to_matrix(tip[3:])[:, 2]
        relative = r_fruit.T @ (tip[:3] - fruit[:3])
        depth = C.FRUIT_HEIGHT - relative[2]
        radial = np.linalg.norm(relative[:2])
        alignment = float(shaft @ vertical)
        push = -float((tool_v[:3] - fruit_v[:3]) @ vertical)
        loads = np.zeros((3,3))
        pressing = 0 < depth <= C.FRUIT_HEIGHT + .004 and radial < .007 and alignment > .95
        if pressing and not self.separated:
            # Rate-dependent cutting reaction at the actual moving edge.
            # A sharper rim has a smaller loaded area than a solid penetrometer.
            resistance = (9.0 + 2.2*np.exp(-depth/.007)) * np.tanh(max(push,0.)/.005)
            reaction = resistance * vertical
            loads[0] += reaction
            loads[1] -= reaction
            self.peak = max(self.peak, float(resistance))
            if resistance >= 3.5 and depth > self.cut:
                self.work += resistance * (depth-self.cut)
                self.cut = float(depth)
            if self.cut >= C.FRUIT_HEIGHT - .005 and self.work >= .43:
                axial = float((core[:3] - tip[:3]) @ shaft)
                if .024 <= axial <= .054:
                    self.separated = True
                    self.engagement = axial
        if not self.separated:
            desired = fruit[:3] + C.CORE_CENTER * vertical
            link = limited(480.*(desired-core[:3]) + 3.0*(fruit_v[:3]-core_v[:3])
                           + [0.,0.,.013*9.81], 9.)
            loads[1] -= link
            loads[2] += link
        elif self.engagement is not None:
            offset = core[:3] - tip[:3]
            if not (-.01 <= offset @ shaft <= .075) or np.linalg.norm(
                offset - (offset @ shaft)*shaft
            ) > .014:
                self.engagement = None
            else:
                # Finite grip from the cut pulp wedging in the metal bore.
                # Overloading or sliding out of the tube loses this contact.
                desired = tip[:3] + self.engagement * shaft
                local_velocity = tool_v[:3] + np.cross(tool_v[3:], desired-tip[:3])
                contact = limited(350.*(desired-core[:3]) + 2.7*(local_velocity-core_v[:3]),
                                  4.5)
                loads[0] -= contact
                loads[2] += contact
        return loads

    def snapshot(self):
        return {"cut_m": self.cut, "cut_work_j": self.work, "peak_load_n": self.peak,
                "separated": self.separated, "tube_grip": self.engagement is not None}
