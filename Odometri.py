import math
import time
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# =========================
# ====== KONFIGURASI ======
# =========================
BASE_PATH              = "/PioneerP3DX"
RIGHT_JOINT_PATH       = "/rightMotor"
LEFT_JOINT_PATH        = "/leftMotor"
RIGHT_WHEEL_SHAPE_PATH = "/rightMotor/rightWheel"  # primitive cylinder (tab Geometry)

# True  : Python yang memanggil sim.step() → waktu simulasi maju per langkah
# False : Simulasi jalan sendiri; Python hanya "ikut" membaca waktu (polling)
STEPPING = True


# =========================
# ====== UTIL FUNCS  ======
# =========================

def wrap_pi(a: float) -> float:
    """Bungkus sudut ke rentang [-pi, pi) menggunakan atan2."""
    return math.atan2(math.sin(a), math.cos(a))


def read_params_from_scene(sim):
    """
    Read-only parameter dari scene:
      - dt(scene): sim.getSimulationTimeStep()
      - L_half   : 0.5*|y_R - y_L| (di frame base)
      - R        : 0.5 * dims[0] dari getShapeGeomInfo(rightWheel) (dims[0] = diameter)
      - handle   : rJ, lJ, base_h
    """
    base_h  = sim.getObject(BASE_PATH)
    rJ      = sim.getObject(RIGHT_JOINT_PATH)
    lJ      = sim.getObject(LEFT_JOINT_PATH)
    rWheel  = sim.getObject(RIGHT_WHEEL_SHAPE_PATH)

    dt_scene = sim.getSimulationTimeStep()

    # L_half dari selisih y_B (frame base)
    pL = sim.getObjectPosition(lJ, base_h)  # [x_B, y_B, z_B]
    pR = sim.getObjectPosition(rJ, base_h)
    L_half = 0.5 * abs(pR[1] - pL[1])       # lateral P3DX = sumbu y_B

    # Radius R dari primitive cylinder: dims[0] = diameter
    result, pureType, dims = sim.getShapeGeomInfo(rWheel)
    if not isinstance(dims, (list, tuple)) or len(dims) < 1:
        raise RuntimeError("getShapeGeomInfo tidak mengembalikan dims yang valid.")
    R = 0.5 * float(dims[0])

    return R, L_half, dt_scene, rJ, lJ, base_h

def get_gt_pose2d(sim, base_h):
    """Ambil pose Ground-Truth (world): (x,y,yaw)."""
    x, y, _ = sim.getObjectPosition(base_h, -1)
    ax, ay, az = sim.getObjectOrientation(base_h, -1)  # yaw = az
    return x, y, wrap_pi(az)


# =========================
# === ODOMETRY + GT LOOP ==
# =========================
def run_until_stop(sim, rJ, lJ, base_h, R, L_half, log_to_sim=True, stepping=True):
    """
    Loop mengikuti status & waktu simulasi:
      - STOP  : state == sim.simulation_stopped        -> break → plot
      - PAUSE : (state & sim.simulation_paused) != 0   -> idle (tidak integrasi)
      - RUN   : selain itu; integrasi hanya ketika waktu simulasi maju
                (stepping=True  -> sim.step() dulu; stepping=False -> polling waktu)
    """
    # ---- ODO lokal ----
    x_o = 0.0
    y_o = 0.0
    th_o = 0.0

    # Pose awal GT (world) → untuk merelatifkan GT
    x0, y0, th0 = get_gt_pose2d(sim, base_h)
    c0, s0 = math.cos(th0), math.sin(th0)  # rotasi th0 untuk proyeksi world->lokal

    # Histori (pakai waktu simulasi)
    t_hist=[]; x_o_hist=[]; y_o_hist=[]; yaw_o_hist=[]      # Histori perkiraaan metode odometri
    x_gt_hist=[]; y_gt_hist=[]; yaw_gt_hist=[]      # Histori posisi dan orientasi ground truth
    ex_hist=[];  ey_hist=[];   eth_hist=[]      # Histori error

    # Waktu simulasi awal
    t_start = sim.getSimulationTime()
    t_prev  = t_start

    if log_to_sim:
        sim.addLog(1, f"[ODO] start (follow sim) | stepping={stepping}")

    while True:
        state = sim.getSimulationState()

        # --- STOP ---
        if state == sim.simulation_stopped:
            if log_to_sim: sim.addLog(1, "[ODO] sim stopped → plotting")
            break

        # --- PAUSE ---
        if (state & sim.simulation_paused) != 0:
            time.sleep(0.02)  # idle ringan
            continue

        # --- RUN / ADVANCING ---
        if stepping:
            # Python yang menggerakkan waktu simulasi
            sim.step()
            t_now = sim.getSimulationTime()
            dt_k  = t_now - t_prev
            if dt_k <= 0.0:
                # (mis. transisi atau belum maju)
                continue
        else:
            # Sim berjalan sendiri; integrasi hanya saat waktu simulasi bertambah
            t_now = sim.getSimulationTime()
            dt_k  = t_now - t_prev
            if dt_k <= 0.0:
                time.sleep(0.01)
                continue

        # ---- ODOMETRI (half-track) ----
        wr = sim.getJointVelocity(rJ)  # rad/s
        wl = sim.getJointVelocity(lJ)
        vr = wr * R
        vl = wl * R
        v  = 0.5 * (vr + vl)
        omega = (vr - vl) / (2.0 * L_half)

        x_o  += v * math.cos(th_o) * dt_k
        y_o  += v * math.sin(th_o) * dt_k
        th_o  = wrap_pi(th_o + omega * dt_k)

        # ---- GT world → relatif pose awal (ke frame lokal) ----
        xw, yw, thw = get_gt_pose2d(sim, base_h)
        dx, dy = xw - x0, yw - y0
        x_gt =  c0*dx + s0*dy
        y_gt =  -s0*dx + c0*dy
        th_gt = wrap_pi(thw - th0)

        # ---- Error ----
        ex = x_o - x_gt
        ey = y_o - y_gt
        eth = wrap_pi(th_o - th_gt)

        # ---- Simpan histori (pakai waktu simulasi relatif) ----
        t_hist.append(t_now - t_start)
        x_o_hist.append(x_o);   y_o_hist.append(y_o);   yaw_o_hist.append(math.degrees(th_o))
        x_gt_hist.append(x_gt); y_gt_hist.append(y_gt); yaw_gt_hist.append(math.degrees(th_gt))
        ex_hist.append(ex);     ey_hist.append(ey);     eth_hist.append(math.degrees(eth))

        t_prev = t_now

    return (t_hist,
            x_o_hist, y_o_hist, yaw_o_hist,
            x_gt_hist, y_gt_hist, yaw_gt_hist,
            ex_hist,  ey_hist,  eth_hist)


# =========================
# ========== MAIN =========
# =========================
def main():
    print("[INFO] connect to CoppeliaSim...")
    client = RemoteAPIClient()
    sim = client.require('sim')

    # Atur stepping sesuai preferensi
    sim.setStepping(STEPPING)
    sim.startSimulation()
    try:
        R, L_half, dt_scene, rJ, lJ, base_h = read_params_from_scene(sim)
        print(f"[PARAM] R={R:.4f} m | L(half)={L_half:.4f} m | dt(scene)={dt_scene:.4f} s | stepping={STEPPING}")

        (t,
         x_o, y_o, yaw_o,
         x_gt, y_gt, yaw_gt,
         ex, ey, eth_deg) = run_until_stop(sim, rJ, lJ, base_h, R, L_half,
                                           log_to_sim=True, stepping=STEPPING)
    finally:
        # aman dipanggil walaupun simulasi sudah berhenti
        sim.stopSimulation()

    # =========================
    # ====== PLOTTING 1 =======
    # (ODO vs GT dalam 1 window, 4 subplot)
    # =========================
    fig, axs = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)

    # x(t)
    ax = axs[0, 0]
    ax.plot(t, x_o, label="x_odo")
    ax.plot(t, x_gt, label="x_gt")
    ax.set_xlabel("t [s]"); ax.set_ylabel("x [m]")
    ax.set_title("x(t) — ODO vs GT")
    ax.grid(True, alpha=0.3); ax.legend()

    # y(t)
    ax = axs[0, 1]
    ax.plot(t, y_o, label="y_odo")
    ax.plot(t, y_gt, label="y_gt")
    ax.set_xlabel("t [s]"); ax.set_ylabel("y [m]")
    ax.set_title("y(t) — ODO vs GT")
    ax.grid(True, alpha=0.3); ax.legend()

    # yaw(t) [deg]
    ax = axs[1, 0]
    ax.plot(t, yaw_o, label="yaw_odo [deg]")
    ax.plot(t, yaw_gt, label="yaw_gt [deg]")
    ax.set_xlabel("t [s]"); ax.set_ylabel("yaw [deg]")
    ax.set_title("yaw(t) — ODO vs GT")
    ax.grid(True, alpha=0.3); ax.legend()

    # lintasan x vs y
    ax = axs[1, 1]
    ax.plot(x_o, y_o, label="ODO")
    ax.plot(x_gt, y_gt, label="GT")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("Trajectory — ODO vs GT")
    ax.grid(True, alpha=0.3); ax.legend()
    ax.set_aspect('equal', 'box')

    # =========================
    # ====== PLOTTING 2 =======
    # (Error curves dalam 1 window terpisah, 3 subplot)
    # =========================
    fig2, axs2 = plt.subplots(3, 1, figsize=(10, 8), sharex=True, constrained_layout=True)

    ax = axs2[0]
    ax.plot(t, ex)
    ax.set_ylabel("e_x [m]")
    ax.set_title("Error x(t)")
    ax.grid(True, alpha=0.3)

    ax = axs2[1]
    ax.plot(t, ey)
    ax.set_ylabel("e_y [m]")
    ax.set_title("Error y(t)")
    ax.grid(True, alpha=0.3)

    ax = axs2[2]
    ax.plot(t, eth_deg)
    ax.set_xlabel("t [s]"); ax.set_ylabel("e_yaw [deg]")
    ax.set_title("Error yaw(t)")
    ax.grid(True, alpha=0.3)

    # Metrik ringkas di console
    if ex and ey:
        rmse_xy = math.sqrt(sum((ex[i]**2 + ey[i]**2) for i in range(len(ex))) / len(ex))
        max_e_yaw = max(abs(v) for v in eth_deg) if eth_deg else 0.0
        print(f"[METRIK] RMSE posisi ~ {rmse_xy:.4f} m | max |e_yaw| ~ {max_e_yaw:.2f} deg")

    plt.show()


if __name__ == "__main__":
    main()
