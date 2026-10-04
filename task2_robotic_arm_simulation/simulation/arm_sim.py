"""
4-DOF robotic arm (+1 gripper DOF): CAD generation, kinematics, and pick-and-place simulation.
Run:  python arm_sim.py        (needs: numpy matplotlib pillow trimesh cadquery)
Outputs go to ../cad, ../simulation, ../screenshots
"""
import os, io, csv, math
import numpy as np
import cadquery as cq
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from PIL import Image

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
CAD, SIM, SHOT = (os.path.join(ROOT, d) for d in ("cad", "simulation", "screenshots"))

# ----------------------------------------------------------------- parameters (metres)
D1, L2, L3, L4 = 0.15, 0.30, 0.25, 0.10      # shoulder height, upper arm, forearm, wrist->TCP
CUBE = 0.05
FIN_X, FIN_Y0 = 0.07, 0.045                   # finger start along wrist x, finger centre offset (open)
QF_MAX = 0.015                                # finger travel to grip the 50 mm cube
LIM = dict(q1=(-170, 170), q2=(-10, 170), q3=(-150, 0), q4=(-120, 120))   # degrees
PICK = np.array([0.30, 0.20, CUBE / 2])
PLACE = np.array([-0.10, 0.35, CUBE / 2 + 0.0005])
HOME = np.array([0.25, 0.0, 0.25])
HOVER = 0.20

# ----------------------------------------------------------------- transforms
def Rz(a):
    c, s = math.cos(a), math.sin(a); M = np.eye(4); M[:2, :2] = [[c, -s], [s, c]]; return M
def Ry(a):
    c, s = math.cos(a), math.sin(a); M = np.eye(4); M[0, 0], M[0, 2], M[2, 0], M[2, 2] = c, s, -s, c; return M
def Tr(x, y, z):
    M = np.eye(4); M[:3, 3] = (x, y, z); return M

# ----------------------------------------------------------------- CAD (CadQuery, link-local frames)
def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0).translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
def cyl(axis, c, r, length):
    plane = {"x": "YZ", "y": "XZ", "z": "XY"}[axis]
    return cq.Workplane(plane).circle(r).extrude(length / 2, both=True).translate(c)

def build_links():
    L = {}
    L["base"] = cyl("z", (0, 0, 0.02), 0.12, 0.04).union(cyl("z", (0, 0, 0.045), 0.07, 0.01))
    L["turret"] = (cyl("z", (0, 0, 0.095), 0.06, 0.11).union(box(-0.05, 0.05, -0.05, 0.05, 0.10, 0.15))
                   .union(cyl("y", (0, 0, D1), 0.05, 0.12)))
    L["upper_arm"] = box(0, L2, -0.03, 0.03, -0.03, 0.03).union(cyl("y", (0, 0, 0), 0.04, 0.09)).union(cyl("y", (L2, 0, 0), 0.04, 0.09))
    L["forearm"] = box(0, L3, -0.025, 0.025, -0.025, 0.025).union(cyl("y", (L3, 0, 0), 0.035, 0.08))
    L["wrist_gripper"] = (cyl("x", (0.03, 0, 0), 0.025, 0.06).union(box(0.06, FIN_X, -0.06, 0.06, -0.02, 0.02)))
    L["finger"] = box(0, 0.05, -0.005, 0.005, -0.015, 0.015)
    return L

COLORS = {"base": (0.25, 0.25, 0.28), "turret": (0.15, 0.35, 0.65), "upper_arm": (0.95, 0.55, 0.10),
          "forearm": (0.95, 0.75, 0.15), "wrist_gripper": (0.30, 0.65, 0.40), "finger": (0.55, 0.55, 0.60),
          "cube": (0.85, 0.15, 0.15)}

def mm(s):
    """Model is built in metres; CAD files are exported in millimetres (SolidWorks/AutoCAD default)."""
    return cq.Workplane("XY").newObject([s.val().scale(1000.0)])

def export_cad(links):
    for n, s in links.items():
        cq.exporters.export(mm(s), os.path.join(CAD, f"{n}.stl"), tolerance=0.5, angularTolerance=0.1)
        cq.exporters.export(mm(s), os.path.join(CAD, f"{n}.step"))

# ----------------------------------------------------------------- kinematics
def fk(q, qf=0.0):
    q1, q2, q3, q4 = q
    T = {"base": np.eye(4)}
    T["turret"] = Rz(q1)
    T["upper_arm"] = T["turret"] @ Tr(0, 0, D1) @ Ry(-q2)
    T["forearm"] = T["upper_arm"] @ Tr(L2, 0, 0) @ Ry(-q3)
    T["wrist_gripper"] = T["forearm"] @ Tr(L3, 0, 0) @ Ry(-q4)
    T["finger_l"] = T["wrist_gripper"] @ Tr(FIN_X, FIN_Y0 - qf, 0)
    T["finger_r"] = T["wrist_gripper"] @ Tr(FIN_X, -FIN_Y0 + qf, 0)
    return T
def tcp(q):
    return (fk(q)["wrist_gripper"] @ np.array([L4, 0, 0, 1]))[:3]

def ik(p):
    """Tool pointing straight down, elbow-up. Returns (q1..q4) in radians."""
    x, y, z = p
    q1 = math.atan2(y, x); r = math.hypot(x, y)
    wz = z + L4 - D1
    c3 = (r * r + wz * wz - L2 ** 2 - L3 ** 2) / (2 * L2 * L3)
    if abs(c3) > 1: raise ValueError(f"target {p} out of reach")
    q3 = -math.acos(c3)
    q2 = math.atan2(wz, r) - math.atan2(L3 * math.sin(q3), L2 + L3 * math.cos(q3))
    q4 = -math.pi / 2 - q2 - q3
    q = np.array([q1, q2, q3, q4])
    for v, k in zip(np.degrees(q), ("q1", "q2", "q3", "q4")):
        assert LIM[k][0] <= v <= LIM[k][1], f"{k}={v:.1f} outside limits"
    assert np.allclose(tcp(q), p, atol=1e-6)
    return q

# ----------------------------------------------------------------- trajectory (pick and place)
def minjerk(s): return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5

def plan(dt=0.05):
    above = lambda p: np.array([p[0], p[1], HOVER])
    segs = [("1 Approach pick", "move", above(PICK), 1.6), ("2 Descend", "move", PICK, 1.0),
            ("3 Close gripper", "grip", QF_MAX, 0.6), ("4 Lift", "move", above(PICK), 1.0),
            ("5 Transfer", "move", above(PLACE), 2.0), ("6 Lower", "move", PLACE, 1.0),
            ("7 Open gripper", "grip", 0.0, 0.6), ("8 Retreat", "move", above(PLACE), 1.0),
            ("9 Return home", "move", HOME, 1.6)]
    frames, p, qf, t = [], HOME.copy(), 0.0, 0.0
    cube_rel, cube_world = None, Tr(*PICK) @ Rz(math.atan2(PICK[1], PICK[0]))
    for name, kind, tgt, dur in segs:
        n = int(round(dur / dt)); p0, qf0 = p.copy(), qf
        for i in range(1, n + 1):
            s = minjerk(i / n); t += dt
            if kind == "move": p_i = p0 + (tgt - p0) * s; qf_i = qf
            else: p_i = p; qf_i = qf0 + (tgt - qf0) * s
            q = ik(p_i); T = fk(q, qf_i)["wrist_gripper"]
            if kind == "grip" and tgt > 0 and i == n: cube_rel = np.linalg.inv(T) @ cube_world   # grasp
            if kind == "grip" and tgt == 0 and i == 1 and cube_rel is not None:                  # release
                cube_world = T @ cube_rel; cube_rel = None
            cw = (T @ cube_rel) if cube_rel is not None else cube_world
            frames.append(dict(t=t, phase=name, q=q, qf=qf_i, tcp=p_i.copy(), cube=cw))
        if kind == "move": p = tgt.copy()
        else: qf = tgt
    return frames

# ----------------------------------------------------------------- rendering
class Renderer:
    def __init__(self, links):
        self.mesh = {}
        for n in links:
            m = trimesh.load(os.path.join(CAD, f"{n}.stl")); self.mesh[n] = (np.array(m.vertices) * 0.001, np.array(m.faces))
        self.mesh["finger_l"] = self.mesh["finger_r"] = self.mesh["finger"]
        cm = trimesh.creation.box(extents=[CUBE] * 3)
        self.mesh["cube"] = (np.array(cm.vertices), np.array(cm.faces))
        self.light = np.array([0.4, -0.5, 0.8]); self.light /= np.linalg.norm(self.light)

    def tris(self, T, cube):
        P, C = [], []
        for n, M in list(T.items()) + [("cube", cube)]:
            if n == "finger": continue
            V, F = self.mesh[n]; W = V @ M[:3, :3].T + M[:3, 3]; tri = W[F]
            nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); nrm /= (np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12)
            sh = 0.40 + 0.60 * np.abs(nrm @ self.light)
            base = np.array(COLORS["finger" if n.startswith("finger") else n])
            P.append(tri); C.append(np.clip(sh[:, None] * base, 0, 1))
        return np.concatenate(P), np.concatenate(C)

    def draw(self, ax, q, qf, cube, view=(28, -60), title="", info=""):
        ax.clear(); ax.computed_zorder = False
        ax.add_collection3d(Poly3DCollection([[(-0.35, -0.15, 0), (0.55, -0.15, 0), (0.55, 0.55, 0), (-0.35, 0.55, 0)]],
                            facecolors="#e8e8e8", edgecolors="#bbbbbb", zorder=0))
        for c, col in ((PICK, "#2e8b57"), (PLACE, "#1f5fbf")):
            x, y = c[0], c[1]; h = 0.032
            ax.add_collection3d(Poly3DCollection([[(x - h, y - h, 0.0005), (x + h, y - h, 0.0005), (x + h, y + h, 0.0005), (x - h, y + h, 0.0005)]],
                                facecolors=col, alpha=0.45, edgecolors=col, zorder=1))
        P, C = self.tris(fk(q, qf), cube)
        ax.add_collection3d(Poly3DCollection(P, facecolors=C, edgecolors=C, linewidths=0.2, zorder=5))
        ax.set_xlim(-0.35, 0.55); ax.set_ylim(-0.15, 0.55); ax.set_zlim(0, 0.5)
        ax.set_box_aspect((0.9, 0.7, 0.5), zoom=1.12); ax.view_init(*view)
        ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
        ax.set_title(title, fontsize=10)
        if info: ax.text2D(0.02, 0.90, info, transform=ax.transAxes, fontsize=8, family="monospace")

def info_text(f):
    d = np.degrees(f["q"])
    return (f"t={f['t']:5.2f}s  {f['phase']}\nq1={d[0]:6.1f} q2={d[1]:6.1f} q3={d[2]:6.1f} q4={d[3]:6.1f} deg\n"
            f"gripper={2*(FIN_Y0-0.005-f['qf'])*1000:4.0f} mm  TCP=({f['tcp'][0]:.2f},{f['tcp'][1]:.2f},{f['tcp'][2]:.2f}) m")

def screenshot(R, f, name, view=(28, -60), title=None):
    fig = plt.figure(figsize=(8, 6.4), dpi=110); ax = fig.add_subplot(111, projection="3d")
    R.draw(ax, f["q"], f["qf"], f["cube"], view, title or f["phase"], info_text(f))
    fig.tight_layout(); fig.savefig(os.path.join(SHOT, name)); plt.close(fig)

# ----------------------------------------------------------------- main
if __name__ == "__main__":
    links = build_links(); export_cad(links)
    R = Renderer(links); frames = plan()
    print("frames:", len(frames), "duration: %.2f s" % frames[-1]["t"])

    q_ready = np.radians([0, 60, -100, -50]); Tq = fk(q_ready, 0.0)
    assy = cq.Assembly(name="robotic_arm")
    def loc(M): return cq.Location(cq.Plane(origin=tuple(M[:3, 3] * 1000), xDir=tuple(M[:3, 0]), normal=tuple(M[:3, 2])))
    for n in ("base", "turret", "upper_arm", "forearm", "wrist_gripper"):
        assy.add(mm(links[n]), name=n, loc=loc(Tq[n]), color=cq.Color(*COLORS[n]))
    for n in ("finger_l", "finger_r"):
        assy.add(mm(links["finger"]), name=n, loc=loc(Tq[n]), color=cq.Color(*COLORS["finger"]))
    assy.save(os.path.join(CAD, "robotic_arm_assembly.step"))
    parts = []
    for n, M in Tq.items():
        V, F = R.mesh[n]; parts.append(trimesh.Trimesh(1000 * (V @ M[:3, :3].T + M[:3, 3]), F))
    trimesh.util.concatenate(parts).export(os.path.join(CAD, "robotic_arm_assembly.stl"))

    with open(os.path.join(SIM, "trajectory.csv"), "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(["t_s", "phase", "q1_deg", "q2_deg", "q3_deg", "q4_deg", "finger_travel_mm", "tcp_x_m", "tcp_y_m", "tcp_z_m"])
        for f in frames:
            d = np.degrees(f["q"]); w.writerow([f"{f['t']:.2f}", f["phase"], *[f"{v:.2f}" for v in d], f"{f['qf']*1000:.2f}", *[f"{v:.4f}" for v in f["tcp"]]])
    Qd = np.degrees(np.array([f["q"] for f in frames]))
    print("joint ranges used (deg):", {k: (round(Qd[:, i].min(), 1), round(Qd[:, i].max(), 1)) for i, k in enumerate(("q1", "q2", "q3", "q4"))})

    def at(phase): return [f for f in frames if f["phase"] == phase][-1]
    home = dict(t=0.0, phase="0 Home", q=ik(HOME), qf=0.0, tcp=HOME, cube=Tr(*PICK) @ Rz(math.atan2(PICK[1], PICK[0])))
    tr = [f for f in frames if f["phase"] == "5 Transfer"]
    keys = [("01_home.png", home), ("02_above_pick.png", at("1 Approach pick")), ("03_grasp.png", at("3 Close gripper")),
            ("04_lift.png", at("4 Lift")), ("05_transfer_midway.png", tr[len(tr) // 2]),
            ("06_place.png", at("6 Lower")), ("07_release.png", at("7 Open gripper")), ("08_return_home.png", at("9 Return home"))]
    for fn, f in keys: screenshot(R, f, fn)
    for fn, v, t in (("09_view_top.png", (90, -90), "Top view"), ("10_view_front.png", (0, -90), "Front view"),
                     ("11_view_side.png", (0, 0), "Side view"), ("12_view_iso.png", (28, -60), "Isometric view")):
        screenshot(R, home, fn, v, f"{t} (home pose)")

    t = np.array([f["t"] for f in frames])
    fig, ax = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    for i, n in enumerate(("q1 base yaw", "q2 shoulder", "q3 elbow", "q4 wrist")): ax[0].plot(t, Qd[:, i], label=n)
    ax[0].set_ylabel("Joint angle (deg)"); ax[0].legend(ncol=4, fontsize=8); ax[0].grid(alpha=.3); ax[0].set_title("Joint trajectories - pick and place")
    ax[1].plot(t, [f["qf"] * 1000 for f in frames], "k"); ax[1].set_ylabel("Finger travel (mm)"); ax[1].set_xlabel("Time (s)"); ax[1].grid(alpha=.3)
    seen = set()
    for f in frames:
        if f["phase"] not in seen:
            seen.add(f["phase"])
            for a in ax: a.axvline(f["t"] - 0.05, color="gray", lw=.5, ls="--")
            ax[1].text(f["t"], 12, f["phase"][0], fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(SHOT, "13_joint_angles.png"), dpi=110); plt.close(fig)
    X = np.array([f["tcp"] for f in frames])
    fig = plt.figure(figsize=(8, 6)); ax = fig.add_subplot(111, projection="3d")
    ax.plot(X[:, 0], X[:, 1], X[:, 2], "r-"); ax.scatter(*PICK, c="g", s=50, label="pick"); ax.scatter(*PLACE, c="b", s=50, label="place"); ax.scatter(*HOME, c="k", s=50, label="home")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)"); ax.legend(); ax.set_title("TCP path"); ax.view_init(28, -60)
    fig.tight_layout(); fig.savefig(os.path.join(SHOT, "14_tcp_path.png"), dpi=110); plt.close(fig)

    fig = plt.figure(figsize=(6.4, 5.2), dpi=100); ax = fig.add_subplot(111, projection="3d"); imgs = []
    for f in frames[::2]:
        R.draw(ax, f["q"], f["qf"], f["cube"], (28, -60), "4-DOF arm: pick and place", info_text(f)); fig.tight_layout()
        b = io.BytesIO(); fig.savefig(b, format="png"); b.seek(0); imgs.append(Image.open(b).convert("P", palette=Image.ADAPTIVE))
    imgs[0].save(os.path.join(SIM, "pick_and_place.gif"), save_all=True, append_images=imgs[1:], duration=100, loop=0)
    print("done")
