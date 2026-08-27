"""
06_simulate_and_compare.py - Nihai Dogrulama: Simulasyon vs Gercek Robot
============================================================================

KULLANIM:
  python scripts/06_simulate_and_compare.py
  python scripts/06_simulate_and_compare.py --data trajectory_log_highspeed.csv
  python scripts/06_simulate_and_compare.py --model models_with_friction_only

--data   : data/raw/ icindeki hangi CSV ile test edilecek (varsayilan:
           ilk gecerli dosya)
--model  : "baseline" (hic tanimlama yok), "friction" (sadece surtunme),
           "full" (surtunme+servo kazanci, varsayilan)
"""

import argparse
import os
import sys

import numpy as np
import pandas as pd
import mujoco

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src import config
from src import common


def get_qpos_addresses(model, joint_names):
    """Her eklemin qpos dizisindeki GERCEK adresini bulur - "ilk N eleman"
    diye varsaymak yerine (bu, serbest taban eklemi gibi ekstra
    serbestlik dereceleri olan robotlarda YANLIS olurdu)."""
    addrs = []
    for jn in joint_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        addrs.append(model.jnt_qposadr[joint_id])
    return addrs


def simulate_trajectory(mjcf_path, target_q_seq, control_dt, initial_qpos, qpos_addrs, act_ids):
    model = mujoco.MjModel.from_xml_path(mjcf_path)
    data = mujoco.MjData(model)
    for k, addr in enumerate(qpos_addrs):
        data.qpos[addr] = initial_qpos[k]
    mujoco.mj_forward(model, data)
    steps = int(round(control_dt / model.opt.timestep))
    simulated_q = np.zeros_like(target_q_seq)
    for i, tq in enumerate(target_q_seq):
        for k, act_id in enumerate(act_ids):
            data.ctrl[act_id] = tq[k]
        for _ in range(steps):
            mujoco.mj_step(model, data)
        for k, addr in enumerate(qpos_addrs):
            simulated_q[i, k] = data.qpos[addr]
    return simulated_q


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", default=None, help="data/raw/ icindeki CSV dosya adi")
    parser.add_argument("--model", default="full", choices=["baseline", "friction", "full"])
    args = parser.parse_args()

    joint_names = common.get_joint_names(config.URDF_PATH)
    n_joints = len(joint_names)

    if args.data:
        csv_path = os.path.join(config.DATA_DIR, args.data)
    else:
        required_cols = common.make_required_columns_servo(n_joints)
        valid = common.discover_csv_files(required_cols, verbose=False)
        csv_path = valid[0]
        print(f"[bilgi] --data verilmedi, ilk gecerli dosya kullaniliyor: {os.path.basename(csv_path)}")

    model_map = {
        "baseline": config.MJCF_PATH,
        "friction": config.MJCF_WITH_FRICTION,
        "full": config.MJCF_WITH_FRICTION_AND_GAINS,
    }
    mjcf_improved = model_map[args.model]

    df = pd.read_csv(csv_path)
    target_q = df[[f"target_q{i+1}" for i in range(n_joints)]].to_numpy()
    actual_q_real = df[[f"actual_q{i+1}" for i in range(n_joints)]].to_numpy()
    control_dt = 0.01
    initial_qpos = actual_q_real[0]

    print(f"[bilgi] Veri: {csv_path}")
    print(f"[bilgi] Karsilastirilan model: {mjcf_improved} (mod: {args.model})")

    # qpos adresleri ve aktuator id'leri, HER IKI model (baseline ve
    # improved) icin ayri ayri bulunmali, cunku farkli MJCF dosyalari
    # olsalar bile - normalde ayni robotu tanimladiklari icin ayni
    # eklem/aktuator yapisina sahip olmalilar.
    model_baseline = mujoco.MjModel.from_xml_path(config.MJCF_PATH)
    qpos_addrs = get_qpos_addresses(model_baseline, joint_names)
    act_ids = [common.find_position_actuator_for_joint(model_baseline, jn) for jn in joint_names]

    print("[bilgi] Simulasyon 1/2: BASELINE (hic tanimlama yok)...")
    sim_baseline = simulate_trajectory(config.MJCF_PATH, target_q, control_dt, initial_qpos,
                                        qpos_addrs, act_ids)

    print(f"[bilgi] Simulasyon 2/2: {args.model.upper()}...")
    sim_improved = simulate_trajectory(mjcf_improved, target_q, control_dt, initial_qpos,
                                        qpos_addrs, act_ids)

    print("\n=== KARSILASTIRMA: Gercek robot vs Simulasyon ===")
    for j in range(n_joints):
        rms_baseline = np.sqrt(np.mean((sim_baseline[:, j] - actual_q_real[:, j]) ** 2))
        rms_improved = np.sqrt(np.mean((sim_improved[:, j] - actual_q_real[:, j]) ** 2))
        if rms_improved < rms_baseline:
            pct = (1 - rms_improved / rms_baseline) * 100
            yorum = f"IYILESTI (%{pct:.0f})"
        else:
            pct = (rms_improved / rms_baseline - 1) * 100
            yorum = f"KOTULESTI (%{pct:.0f}) - dikkat!"
        print(f"  {joint_names[j]}: baseline={rms_baseline:.5f} rad | {args.model}={rms_improved:.5f} rad -> {yorum}")


if __name__ == "__main__":
    main()
