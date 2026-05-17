"""
Robotics Project Final Phase - Spring 2025/26
Torque Trajectory Computation for the Dobot CR10 Using Pinocchio

Pipeline:
  Part A  - Cartesian trajectory generation + Inverse Kinematics  -> q(t)
  Part B  - Jacobian-based joint velocities and accelerations     -> q_dot(t), q_ddot(t)
  Part C  - Inverse dynamics (RNEA) via Pinocchio                 -> tau(t)
"""

import numpy as np
import pinocchio as pin
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

# ─────────────────────────────────────────────────────────────────────────────
# 0.  LOAD THE DOBOT CR10 MODEL
# ─────────────────────────────────────────────────────────────────────────────
URDF_PATH = 'dobot_cr10.urdf'
model = pin.buildModelFromUrdf(URDF_PATH)
data  = model.createData()

EE_FRAME_NAME = 'ee_link'
EE_FRAME_ID   = model.getFrameId(EE_FRAME_NAME)
N_JOINTS      = model.nv

print("=" * 60)
print("Dobot CR10 Model Loaded")
print(f"  Degrees of freedom : {N_JOINTS}")
print(f"  Joints             : {[model.names[i] for i in range(1, model.njoints)]}")
print(f"  End-effector frame : {EE_FRAME_NAME} (id={EE_FRAME_ID})")
print("=" * 60)


# ─────────────────────────────────────────────────────────────────────────────
# HELPER: Forward Kinematics
# ─────────────────────────────────────────────────────────────────────────────
def fk_position(q):
    """Return the 3-D EE position for joint config q."""
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    return data.oMf[EE_FRAME_ID].translation.copy()


# ─────────────────────────────────────────────────────────────────────────────
# HELPER: Translational Jacobian (world-aligned local frame)
# ─────────────────────────────────────────────────────────────────────────────
def compute_jacobian(q):
    """Return the 3×n translational Jacobian in LOCAL_WORLD_ALIGNED frame."""
    pin.computeJointJacobians(model, data, q)
    J_full = pin.getFrameJacobian(
        model, data, EE_FRAME_ID, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
    )
    return J_full[:3, :]   # translational rows


# ─────────────────────────────────────────────────────────────────────────────
# HELPER: Jacobian time derivative (translational rows only)
# ─────────────────────────────────────────────────────────────────────────────
def compute_jacobian_dot(q, q_dot):
    """Return the 3×n time-derivative of the translational Jacobian."""
    pin.computeJointJacobiansTimeVariation(model, data, q, q_dot)
    Jdot_full = pin.getFrameJacobianTimeVariation(
        model, data, EE_FRAME_ID, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
    )
    return Jdot_full[:3, :]


# ─────────────────────────────────────────────────────────────────────────────
# HELPER: Damped pseudo-inverse
# ─────────────────────────────────────────────────────────────────────────────
LAMBDA = 0.01    # damping factor

def damped_pinv(J):
    """Damped least-squares pseudoinverse: J^T (J J^T + λ²I)^{-1}."""
    lam2 = LAMBDA ** 2
    m = J.shape[0]
    return J.T @ np.linalg.inv(J @ J.T + lam2 * np.eye(m))


# ─────────────────────────────────────────────────────────────────────────────
# HELPER: Numerical Inverse Kinematics (damped Jacobian iterations)
# ─────────────────────────────────────────────────────────────────────────────
IK_TOL       = 1e-5
IK_MAX_ITER  = 500
IK_ALPHA     = 0.8    # step size

def inverse_kinematics(x_d, q_init):
    """
    Compute joint angles q such that FK(q) ≈ x_d using
    damped Jacobian pseudoinverse IK.
    """
    q = q_init.copy()
    for _ in range(IK_MAX_ITER):
        x_cur = fk_position(q)
        err   = x_d - x_cur
        if np.linalg.norm(err) < IK_TOL:
            break
        J   = compute_jacobian(q)
        dq  = damped_pinv(J) @ err
        q   = q + IK_ALPHA * dq
        # clip to joint limits
        q = np.clip(q, model.lowerPositionLimit, model.upperPositionLimit)
    return q


# ─────────────────────────────────────────────────────────────────────────────
# PART A – TRAJECTORY GENERATION
# ─────────────────────────────────────────────────────────────────────────────

def fifth_order_poly(t, T):
    """Return s(t), s_dot(t), s_ddot(t) for 5th-order time scaling."""
    tau  = t / T
    s     = 10*tau**3 - 15*tau**4 + 6*tau**5
    s_d   = (30*tau**2 - 60*tau**3 + 30*tau**4) / T
    s_dd  = (60*tau  - 180*tau**2 + 120*tau**3) / T**2
    return s, s_d, s_dd


def generate_straight_line(x_A, x_B, T, dt):
    """
    Generate straight-line Cartesian trajectory from x_A to x_B.
    Returns dicts with arrays: pos, vel, acc (each shape Nx3).
    """
    t_vec = np.arange(0, T + dt, dt)
    N = len(t_vec)
    Dx = x_B - x_A
    pos = np.zeros((N, 3))
    vel = np.zeros((N, 3))
    acc = np.zeros((N, 3))
    for k, t in enumerate(t_vec):
        s, sd, sdd = fifth_order_poly(t, T)
        pos[k] = x_A + s   * Dx
        vel[k] =        sd  * Dx
        acc[k] =        sdd * Dx
    return t_vec, pos, vel, acc


def generate_circular(x_c, R, T, dt):
    """
    Generate circular trajectory (horizontal plane) centred at x_c.
    Returns dicts with arrays: pos, vel, acc (each shape Nx3).
    """
    t_vec = np.arange(0, T + dt, dt)
    N = len(t_vec)
    pos = np.zeros((N, 3))
    vel = np.zeros((N, 3))
    acc = np.zeros((N, 3))
    for k, t in enumerate(t_vec):
        s, sd, sdd = fifth_order_poly(t, T)
        theta    = 2*np.pi * s
        theta_d  = 2*np.pi * sd
        theta_dd = 2*np.pi * sdd
        ct, st   = np.cos(theta), np.sin(theta)
        pos[k]   = [x_c[0] + R*ct,       x_c[1] + R*st,       x_c[2]]
        vel[k]   = [-R*st*theta_d,         R*ct*theta_d,         0.0]
        acc[k]   = [-R*ct*theta_d**2 - R*st*theta_dd,
                     -R*st*theta_d**2 + R*ct*theta_dd,
                     0.0]
    return t_vec, pos, vel, acc


# ─────────────────────────────────────────────────────────────────────────────
# PART A  STEP A5 – Solve IK for every waypoint
# ─────────────────────────────────────────────────────────────────────────────

def compute_joint_positions(cart_pos, q_init):
    """Run IK at each time step.  Returns q_traj (N×n) and tracking error."""
    N = len(cart_pos)
    q_traj = np.zeros((N, N_JOINTS))
    err_list = []
    q = q_init.copy()
    for k in range(N):
        q = inverse_kinematics(cart_pos[k], q)
        q_traj[k] = q
        err_list.append(np.linalg.norm(fk_position(q) - cart_pos[k]))
    print(f"  IK max position error: {max(err_list)*1000:.3f} mm")
    return q_traj


# ─────────────────────────────────────────────────────────────────────────────
# PART B – JACOBIAN-BASED JOINT VELOCITIES AND ACCELERATIONS
# ─────────────────────────────────────────────────────────────────────────────

def compute_joint_velocities(q_traj, cart_vel):
    """q_dot = J†(q) * x_dot."""
    N = len(q_traj)
    qd_traj = np.zeros((N, N_JOINTS))
    for k in range(N):
        q = q_traj[k]
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        J  = compute_jacobian(q)
        qd_traj[k] = damped_pinv(J) @ cart_vel[k]
    return qd_traj


def compute_joint_accelerations(q_traj, qd_traj, cart_vel, cart_acc):
    """q_ddot = J†(q) * [x_ddot - J_dot(q,q_dot)*q_dot]."""
    N = len(q_traj)
    qdd_traj = np.zeros((N, N_JOINTS))
    for k in range(N):
        q  = q_traj[k]
        qd = qd_traj[k]
        pin.forwardKinematics(model, data, q, qd)
        pin.updateFramePlacements(model, data)
        J     = compute_jacobian(q)
        Jdot  = compute_jacobian_dot(q, qd)
        rhs   = cart_acc[k] - Jdot @ qd
        qdd_traj[k] = damped_pinv(J) @ rhs
    return qdd_traj


# ─────────────────────────────────────────────────────────────────────────────
# PART C – TORQUE COMPUTATION VIA PINOCCHIO RNEA
# ─────────────────────────────────────────────────────────────────────────────

def compute_torques(q_traj, qd_traj, qdd_traj):
    """tau = RNEA(q, q_dot, q_ddot)  applied at every time step."""
    N = len(q_traj)
    tau_traj = np.zeros((N, N_JOINTS))
    for k in range(N):
        tau_traj[k] = pin.rnea(model, data, q_traj[k], qd_traj[k], qdd_traj[k])
    return tau_traj


# ─────────────────────────────────────────────────────────────────────────────
# PLOTTING HELPERS
# ─────────────────────────────────────────────────────────────────────────────
JOINT_LABELS = [f'Joint {i+1}' for i in range(N_JOINTS)]
COLORS       = plt.cm.tab10(np.linspace(0, 1, N_JOINTS))


def plot_cartesian_traj(t, pos, title, ax3d=None, fig=None):
    fig2, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    labels = ['x (m)', 'y (m)', 'z (m)']
    clrs   = ['steelblue', 'darkorange', 'forestgreen']
    for i, (ax, lbl, c) in enumerate(zip(axes, labels, clrs)):
        ax.plot(t, pos[:, i], color=c, lw=1.8)
        ax.set_ylabel(lbl, fontsize=11)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel('Time (s)', fontsize=11)
    fig2.suptitle(title, fontsize=13, fontweight='bold')
    plt.tight_layout()
    return fig2


def plot_joint_trajectories(t, data_dict, ylabel, title):
    """Plot several joint signals on one figure (one subplot per joint)."""
    fig, axes = plt.subplots(3, 2, figsize=(13, 10), sharex=True)
    axes = axes.flatten()
    for i, ax in enumerate(axes):
        for label, arr in data_dict.items():
            ax.plot(t, arr[:, i], lw=1.6, label=label)
        ax.set_title(JOINT_LABELS[i], fontsize=10)
        ax.set_ylabel(ylabel, fontsize=9)
        ax.grid(True, alpha=0.3)
        if i >= 4:
            ax.set_xlabel('Time (s)', fontsize=9)
        if len(data_dict) > 1 and i == 0:
            ax.legend(fontsize=8)
    fig.suptitle(title, fontsize=13, fontweight='bold')
    plt.tight_layout()
    return fig


def plot_3d_path(pos_list, labels, title):
    fig = plt.figure(figsize=(9, 7))
    ax  = fig.add_subplot(111, projection='3d')
    clrs = ['steelblue', 'darkorange']
    for pos, lbl, c in zip(pos_list, labels, clrs):
        ax.plot(pos[:, 0], pos[:, 1], pos[:, 2], color=c, lw=2, label=lbl)
        ax.scatter(*pos[0],  color=c, s=60, marker='o')
        ax.scatter(*pos[-1], color=c, s=60, marker='^')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title(title, fontsize=12, fontweight='bold')
    ax.legend()
    plt.tight_layout()
    return fig


# ─────────────────────────────────────────────────────────────────────────────
# MAIN PIPELINE
# ─────────────────────────────────────────────────────────────────────────────
OUTPUT_DIR = '/Users/USER/Desktop/Project/Plots'

def run_pipeline(T, dt, traj_type, label_suffix=''):
    """Full pipeline for one trajectory type and motion duration."""
    print(f"\n{'='*60}")
    print(f"  Trajectory: {traj_type.upper()}  |  T = {T} s  |  dt = {dt} s")
    print(f"{'='*60}")

    dt = float(dt)

    # ── A1: Time vector ──────────────────────────────────────────────────────
    t_vec = np.arange(0.0, T + dt, dt)

    # ── A2-A4: Cartesian trajectory ──────────────────────────────────────────
    if traj_type == 'straight':
        # Two reachable Cartesian points (verified by FK above)
        x_A = np.array([0.85, -0.10, 0.30])
        x_B = np.array([0.65,  0.25, 0.45])
        t_vec, cart_pos, cart_vel, cart_acc = generate_straight_line(x_A, x_B, T, dt)
    elif traj_type == 'circular':
        x_c = np.array([0.75, 0.08, 0.38])
        R   = 0.15
        t_vec, cart_pos, cart_vel, cart_acc = generate_circular(x_c, R, T, dt)
    else:
        raise ValueError(f"Unknown traj_type '{traj_type}'")

    N = len(t_vec)
    print(f"  Time steps: {N}")

    # ── A5: Inverse Kinematics ───────────────────────────────────────────────
    # Home configuration (near first waypoint)
    q_home = np.array([0.0, 1.0, -1.4, 0.0, 0.9, 0.0])
    print("  Running IK ...")
    q_traj = compute_joint_positions(cart_pos, q_home)

    # ── B2-B3: Joint velocities ──────────────────────────────────────────────
    print("  Computing joint velocities ...")
    qd_traj = compute_joint_velocities(q_traj, cart_vel)

    # ── B4-B5: Joint accelerations ───────────────────────────────────────────
    print("  Computing joint accelerations ...")
    qdd_traj = compute_joint_accelerations(q_traj, qd_traj, cart_vel, cart_acc)

    # ── C2: Torques via RNEA ─────────────────────────────────────────────────
    print("  Computing torques via RNEA ...")
    tau_traj = compute_torques(q_traj, qd_traj, qdd_traj)

    print(f"  Max |torque| per joint: "
          f"{['τ'+str(i+1)+'='+str(round(np.max(np.abs(tau_traj[:,i])),1))+'N·m' for i in range(N_JOINTS)]}")

    return t_vec, cart_pos, cart_vel, cart_acc, q_traj, qd_traj, qdd_traj, tau_traj


# ─────────────────────────────────────────────────────────────────────────────
# RUN FOR BOTH TRAJECTORIES × TWO DURATIONS
# ─────────────────────────────────────────────────────────────────────────────
DT       = 0.02        # 50 Hz
T_SLOW   = 6.0         # slower motion
T_FAST   = 3.0         # faster motion
DURATIONS = [T_SLOW, T_FAST]

results = {}
for traj in ['straight', 'circular']:
    results[traj] = {}
    for T in DURATIONS:
        key = f'T{int(T)}'
        (t_vec, cart_pos, cart_vel, cart_acc,
         q_traj, qd_traj, qdd_traj, tau_traj) = run_pipeline(T, DT, traj)
        results[traj][key] = dict(
            t=t_vec, cart_pos=cart_pos, cart_vel=cart_vel, cart_acc=cart_acc,
            q=q_traj, qd=qd_traj, qdd=qdd_traj, tau=tau_traj
        )


# ─────────────────────────────────────────────────────────────────────────────
# GENERATE ALL REQUIRED PLOTS
# ─────────────────────────────────────────────────────────────────────────────
print("\n" + "="*60)
print("  Generating plots ...")

saved_figs = []

for traj_name in ['straight', 'circular']:
    traj_label = 'Straight-Line' if traj_name == 'straight' else 'Circular'

    for T in DURATIONS:
        key = f'T{int(T)}'
        r   = results[traj_name][key]
        t   = r['t']
        suffix = f"{traj_name}_T{int(T)}"

        # ── Plot 1: Desired Cartesian trajectory ─────────────────────────────
        fig = plot_cartesian_traj(
            t, r['cart_pos'],
            f"{traj_label} Trajectory – Cartesian Position  (T={T} s)")
        fname = f'{OUTPUT_DIR}/01_cartesian_{suffix}.png'
        fig.savefig(fname, dpi=120, bbox_inches='tight'); plt.close(fig)
        saved_figs.append(fname)

        # ── Plot 2: Joint positions ───────────────────────────────────────────
        fig = plot_joint_trajectories(
            t, {'q(t)': r['q']}, 'Angle (rad)',
            f"{traj_label} – Joint Positions  (T={T} s)")
        fname = f'{OUTPUT_DIR}/02_joint_pos_{suffix}.png'
        fig.savefig(fname, dpi=120, bbox_inches='tight'); plt.close(fig)
        saved_figs.append(fname)

        # ── Plot 3: Joint velocities ──────────────────────────────────────────
        fig = plot_joint_trajectories(
            t, {'q̇(t)': r['qd']}, 'Velocity (rad/s)',
            f"{traj_label} – Joint Velocities  (T={T} s)")
        fname = f'{OUTPUT_DIR}/03_joint_vel_{suffix}.png'
        fig.savefig(fname, dpi=120, bbox_inches='tight'); plt.close(fig)
        saved_figs.append(fname)

        # ── Plot 4: Joint accelerations ───────────────────────────────────────
        fig = plot_joint_trajectories(
            t, {'q̈(t)': r['qdd']}, 'Acceleration (rad/s²)',
            f"{traj_label} – Joint Accelerations  (T={T} s)")
        fname = f'{OUTPUT_DIR}/04_joint_acc_{suffix}.png'
        fig.savefig(fname, dpi=120, bbox_inches='tight'); plt.close(fig)
        saved_figs.append(fname)

        # ── Plot 5: Joint torques ─────────────────────────────────────────────
        fig = plot_joint_trajectories(
            t, {'τ(t)': r['tau']}, 'Torque (N·m)',
            f"{traj_label} – Joint Torques  (T={T} s)")
        fname = f'{OUTPUT_DIR}/05_joint_torque_{suffix}.png'
        fig.savefig(fname, dpi=120, bbox_inches='tight'); plt.close(fig)
        saved_figs.append(fname)

    # ── Plot 6: Torque comparison for different durations ────────────────────
    data_dict_cmp = {f'T={T} s': results[traj_name][f'T{int(T)}']['tau'] for T in DURATIONS}
    # Resample to common time base (use shorter duration reference)
    # Just overlay (time axes differ), so plot separately per joint
    fig, axes = plt.subplots(3, 2, figsize=(13, 10), sharex=False)
    axes = axes.flatten()
    line_styles = ['-', '--']
    for i, ax in enumerate(axes):
        for (lbl, tau_arr), ls, T in zip(data_dict_cmp.items(), line_styles, DURATIONS):
            t_ref = results[traj_name][f'T{int(T)}']['t']
            ax.plot(t_ref, tau_arr[:, i], lw=1.8, ls=ls, label=lbl)
        ax.set_title(JOINT_LABELS[i], fontsize=10)
        ax.set_ylabel('Torque (N·m)', fontsize=9)
        ax.set_xlabel('Time (s)', fontsize=9)
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.legend(fontsize=9)
    fig.suptitle(f"{traj_label} – Torque Comparison: Different Motion Durations",
                 fontsize=13, fontweight='bold')
    plt.tight_layout()
    fname = f'{OUTPUT_DIR}/06_torque_comparison_{traj_name}.png'
    fig.savefig(fname, dpi=120, bbox_inches='tight'); plt.close(fig)
    saved_figs.append(fname)

# ── Plot 7: 3-D path visualization ───────────────────────────────────────────
fig = plot_3d_path(
    [results['straight']['T6']['cart_pos'],
     results['circular']['T6']['cart_pos']],
    ['Straight-line', 'Circular'],
    'Desired End-Effector Trajectories in 3-D')
fname = f'{OUTPUT_DIR}/07_3d_paths.png'
fig.savefig(fname, dpi=120, bbox_inches='tight'); plt.close(fig)
saved_figs.append(fname)

# ── Plot 8: Max torque vs duration bar chart ──────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(13, 5))
for ax, traj_name, traj_label in zip(axes,
                                      ['straight', 'circular'],
                                      ['Straight-Line', 'Circular']):
    x     = np.arange(N_JOINTS)
    width = 0.35
    max_T6  = np.max(np.abs(results[traj_name]['T6']['tau']),  axis=0)
    max_T3  = np.max(np.abs(results[traj_name]['T3']['tau']),  axis=0)
    ax.bar(x - width/2, max_T6, width, label='T=6 s (slow)', color='steelblue', alpha=0.85)
    ax.bar(x + width/2, max_T3, width, label='T=3 s (fast)', color='darkorange', alpha=0.85)
    ax.set_xticks(x)
    ax.set_xticklabels(JOINT_LABELS, fontsize=10)
    ax.set_ylabel('Peak |Torque| (N·m)', fontsize=11)
    ax.set_title(f'{traj_label} – Peak Torque vs Motion Duration', fontsize=12, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(True, axis='y', alpha=0.3)
plt.tight_layout()
fname = f'{OUTPUT_DIR}/08_peak_torque_comparison.png'
fig.savefig(fname, dpi=120, bbox_inches='tight'); plt.close(fig)
saved_figs.append(fname)

print(f"\n  Saved {len(saved_figs)} figures to {OUTPUT_DIR}")
for f in saved_figs:
    print(f"    {os.path.basename(f)}")