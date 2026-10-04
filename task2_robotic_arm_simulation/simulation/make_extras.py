"""2D DXF drawing, kinematic diagram, and URDF for the 4-DOF arm."""
import os, math
import numpy as np, ezdxf
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from ezdxf.addons.drawing import RenderContext, Frontend
from ezdxf.addons.drawing.matplotlib import MatplotlibBackend
from ezdxf.addons.drawing.config import Configuration, BackgroundPolicy
from arm_sim import D1, L2, L3, L4, FIN_X, FIN_Y0, QF_MAX, LIM, PICK, PLACE, CAD, SHOT, ROOT

mmU = 1000.0
q2, q3, q4 = np.radians([60, -100, -50])          # "ready" pose used for the drawing
S = np.array([0, D1]); a1, a2, a3 = q2, q2 + q3, q2 + q3 + q4
E = S + L2 * np.array([math.cos(a1), math.sin(a1)]); W = E + L3 * np.array([math.cos(a2), math.sin(a2)])
T = W + L4 * np.array([math.cos(a3), math.sin(a3)])

def rect(p0, ang, length, hw):
    d = np.array([math.cos(ang), math.sin(ang)]); n = np.array([-d[1], d[0]]); p1 = p0 + d * length
    return [tuple((p0 + n * hw) * mmU), tuple((p1 + n * hw) * mmU), tuple((p1 - n * hw) * mmU), tuple((p0 - n * hw) * mmU)]

# ---------------------------------------------------------------- DXF (mm)
doc = ezdxf.new("R2010"); doc.units = 4; msp = doc.modelspace()
for name, col in (("BODY", 5), ("LINKS", 30), ("JOINTS", 1), ("DIM", 3), ("ENVELOPE", 8), ("TEXT", 250)): doc.layers.add(name, color=col)
dx = dict(layer="BODY")
msp.add_lwpolyline([(-120, 0), (120, 0), (120, 40), (70, 40), (70, 55), (60, 55), (60, 150), (-60, 150), (-60, 55), (-70, 55), (-70, 40), (-120, 40)], close=True, dxfattribs=dx)
for pts in (rect(S, a1, L2, 0.03), rect(E, a2, L3, 0.025), rect(W, a3, 0.07, 0.02)):
    msp.add_lwpolyline(pts, close=True, dxfattribs={"layer": "LINKS"})
for p, r, lab in ((S, 50, "J2 shoulder"), (E, 40, "J3 elbow"), (W, 25, "J4 wrist")):
    msp.add_circle(tuple(p * mmU), r, dxfattribs={"layer": "JOINTS"}); msp.add_circle(tuple(p * mmU), 5, dxfattribs={"layer": "JOINTS"})
    msp.add_text(lab, height=14, dxfattribs={"layer": "TEXT", "insert": tuple(p * mmU + np.array([r + 8, 10]))})
msp.add_circle((0, D1 * mmU), 5, dxfattribs={"layer": "JOINTS"})
msp.add_text("J1 base yaw (Z axis)", height=14, dxfattribs={"layer": "TEXT", "insert": (-300, 60)})
msp.add_line((0, -10), (0, 520), dxfattribs={"layer": "DIM", "linetype": "CONTINUOUS"})
for lab, a, b in (("D1 = 150", (-160, 0), (-160, 150)), ("L2 = 300", None, None), ("L3 = 250", None, None), ("L4 = 100 (to TCP)", None, None)):
    if a: msp.add_line(a, b, dxfattribs={"layer": "DIM"}); msp.add_text(lab, height=14, dxfattribs={"layer": "DIM", "insert": (-300, 80)})
msp.add_text("L2 = 300 mm   L3 = 250 mm   L4 = 100 mm (wrist to TCP)   D1 = 150 mm", height=16, dxfattribs={"layer": "TEXT", "insert": (-150, -60)})
msp.add_text("SIDE VIEW - ready pose q2=60, q3=-100, q4=-50 deg", height=18, dxfattribs={"layer": "TEXT", "insert": (-150, -90)})
msp.add_circle(tuple(T * mmU), 6, dxfattribs={"layer": "DIM"}); msp.add_text("TCP", height=14, dxfattribs={"layer": "DIM", "insert": tuple(T * mmU + [10, -5])})
# top view (offset right)
ox = 1000
msp.add_circle((ox, 0), 120, dxfattribs={"layer": "BODY"}); msp.add_circle((ox, 0), 60, dxfattribs={"layer": "BODY"})
msp.add_circle((ox, 0), (L2 + L3) * mmU, dxfattribs={"layer": "ENVELOPE"}); msp.add_circle((ox, 0), (L2 - L3) * mmU + 0, dxfattribs={"layer": "ENVELOPE"})
for c, lab, col in ((PICK, "PICK", 3), (PLACE, "PLACE", 5)):
    msp.add_lwpolyline([(ox + c[0] * mmU + dx_, c[1] * mmU + dy_) for dx_, dy_ in ((-25, -25), (25, -25), (25, 25), (-25, 25))], close=True, dxfattribs={"layer": "DIM", "color": col})
    msp.add_text(lab, height=14, dxfattribs={"layer": "TEXT", "insert": (ox + c[0] * mmU + 30, c[1] * mmU + 30)})
msp.add_text("TOP VIEW - max reach 550 mm from shoulder axis", height=18, dxfattribs={"layer": "TEXT", "insert": (ox - 300, -620)})
msp.add_arc((ox, 0), 200, LIM["q1"][0] * 0 - 170 + 90 - 90, 0, dxfattribs={"layer": "ENVELOPE"}) if False else None
doc.saveas(os.path.join(CAD, "robotic_arm_2D_drawing.dxf"))
fig = plt.figure(figsize=(14, 6)); ax = fig.add_axes([0, 0, 1, 1]); ax.set_facecolor("white")
Frontend(RenderContext(doc), MatplotlibBackend(ax), config=Configuration(background_policy=BackgroundPolicy.WHITE)).draw_layout(msp); ax.set_aspect("equal"); fig.savefig(os.path.join(SHOT, "15_2D_drawing_dxf.png"), dpi=110, facecolor="white"); plt.close(fig)

# ---------------------------------------------------------------- kinematic diagram
fig, ax = plt.subplots(1, 2, figsize=(12, 5.5))
a = ax[0]; pts = np.array([[0, 0], [0, D1], E, W, T])
a.plot(pts[:, 0], pts[:, 1], "-", color="#444", lw=5, solid_capstyle="round"); a.fill_between([-.12, .12], 0, .04, color="#333")
for p, lab, off in ((pts[1], "J2\nshoulder pitch", (-.20, .02)), (pts[2], "J3\nelbow pitch", (.03, .05)), (pts[3], "J4\nwrist pitch", (.04, .02))):
    a.plot(*p, "o", ms=14, mfc="white", mec="r", mew=2); a.annotate(lab, p, xytext=p + np.array(off), fontsize=9, color="r")
a.plot(0, D1, "o", ms=0); a.annotate("J1 base yaw\n(about Z)", (0, 0.04), xytext=(.05, -.07), fontsize=9, color="r")
a.plot(*T, "g*", ms=16); a.annotate("TCP\n(gripper)", T, xytext=T + np.array([.03, -.07]), fontsize=9, color="g")
for (p, q, lab) in ((pts[1], pts[2], "L2=300"), (pts[2], pts[3], "L3=250"), (pts[3], pts[4], "L4=100")):
    m = (p + q) / 2; a.text(m[0] - .1 if lab != "L4=100" else m[0] + .03, m[1] + .03, lab, fontsize=9, color="b")
a.text(-.19, D1 / 2, "D1=150", fontsize=9, color="b"); a.set_aspect("equal"); a.set_xlim(-.3, .55); a.set_ylim(-.1, .55); a.grid(alpha=.3); a.set_xlabel("m"); a.set_title("Kinematic diagram - side view (ready pose)")
b = ax[1]; th = np.linspace(0, 2 * np.pi, 200)
b.fill(0.55 * np.cos(th), 0.55 * np.sin(th), color="#cfe8ff", label="workspace envelope (max reach 550 mm)")
b.plot(*PICK[:2], "gs", ms=12, label="pick"); b.plot(*PLACE[:2], "bs", ms=12, label="place"); b.plot(0, 0, "ko", ms=10)
b.set_aspect("equal"); b.set_xlim(-.6, .6); b.set_ylim(-.6, .6); b.grid(alpha=.3); b.legend(loc="lower right", fontsize=8); b.set_title("Top view - workspace envelope")
fig.tight_layout(); fig.savefig(os.path.join(SHOT, "16_kinematic_diagram.png"), dpi=110); plt.close(fig)

# ---------------------------------------------------------------- URDF
def link(n, color):
    return f'''  <link name="{n}">
    <visual><geometry><mesh filename="{n if n!='finger_l' and n!='finger_r' else 'finger'}.stl" scale="0.001 0.001 0.001"/></geometry><material name="m_{n}"><color rgba="{color} 1"/></material></visual>
    <collision><geometry><mesh filename="{n if n!='finger_l' and n!='finger_r' else 'finger'}.stl" scale="0.001 0.001 0.001"/></geometry></collision>
    <inertial><mass value="0.5"/><inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>'''
def joint(n, typ, par, ch, xyz, axis, lo, hi):
    return f'''  <joint name="{n}" type="{typ}"><parent link="{par}"/><child link="{ch}"/><origin xyz="{xyz}" rpy="0 0 0"/><axis xyz="{axis}"/>
    <limit lower="{lo}" upper="{hi}" effort="50" velocity="2.0"/></joint>'''
r = lambda d: f"{math.radians(d):.5f}"
cols = {"base": "0.25 0.25 0.28", "turret": "0.15 0.35 0.65", "upper_arm": "0.95 0.55 0.10", "forearm": "0.95 0.75 0.15",
        "wrist_gripper": "0.30 0.65 0.40", "finger_l": "0.55 0.55 0.60", "finger_r": "0.55 0.55 0.60"}
urdf = ['<?xml version="1.0"?>', '<robot name="arm4dof">'] + [link(n, c) for n, c in cols.items()] + [
    joint("J1_base_yaw", "revolute", "base", "turret", "0 0 0", "0 0 1", r(LIM["q1"][0]), r(LIM["q1"][1])),
    joint("J2_shoulder", "revolute", "turret", "upper_arm", f"0 0 {D1}", "0 -1 0", r(LIM["q2"][0]), r(LIM["q2"][1])),
    joint("J3_elbow", "revolute", "upper_arm", "forearm", f"{L2} 0 0", "0 -1 0", r(LIM["q3"][0]), r(LIM["q3"][1])),
    joint("J4_wrist", "revolute", "forearm", "wrist_gripper", f"{L3} 0 0", "0 -1 0", r(LIM["q4"][0]), r(LIM["q4"][1])),
    joint("G1_finger_left", "prismatic", "wrist_gripper", "finger_l", f"{FIN_X} {FIN_Y0} 0", "0 -1 0", 0, 0.02),
    joint("G1_finger_right", "prismatic", "wrist_gripper", "finger_r", f"{FIN_X} {-FIN_Y0} 0", "0 1 0", 0, 0.02), "</robot>"]
open(os.path.join(CAD, "robot_arm.urdf"), "w").write("\n".join(urdf))
try:
    import yourdfpy
    u = yourdfpy.URDF.load(os.path.join(CAD, "robot_arm.urdf")); print("URDF loaded OK; actuated joints:", u.actuated_joint_names)
    u.update_cfg(np.radians([0, 60, -100, -50, 0, 0])); print("URDF FK ok, wrist frame z =", round(u.get_transform("wrist_gripper")[2, 3], 4))
except Exception as e: print("URDF check skipped/failed:", e)
