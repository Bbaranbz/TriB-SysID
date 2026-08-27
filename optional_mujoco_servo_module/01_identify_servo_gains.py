"""
04_identify_servo_gains.py - Servo Kazanci (kp, kv) Tanimlama
==================================================================

KULLANIM: python scripts/04_identify_servo_gains.py
(Onceden 03_apply_friction_to_mjcf.py calistirilmis olmali.)

Eklem sayisi ve isimleri URDF'den otomatik okunur. Aktuator eslestirmesi
ISIM DESENINE ("joint_i_position") DEGIL, MuJoCo'nun kendi ic baglanti
yapisina (actuator_trnid) bakarak yapilir - bu yuzden aktuatorunuzu
istediginiz gibi adlandirabilirsiniz, script yine de dogru ekleme
dogru kazanci atar.
"""

import json
import os
import sys

import numpy as np
import mujoco
import pandas as pd
from scipy.optimize import minimize

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src import config
from src import common


def set_servo_gains(model, act_ids, kp_array, kv_array):
    for idx, act_id in enumerate(act_ids):
        model.actuator_gainprm[act_id, 0] = kp_array[idx]
        model.actuator_biasprm[act_id, 1] = -kp_array[idx]
        model.actuator_biasprm[act_id, 2] = -kv_array[idx]


def simulate_fast(model, data, act_ids, target_q_seq, control_dt, initial_qpos):
    mujoco.mj_resetData(model, data)
    data.qpos[: len(initial_qpos)] = initial_qpos
    mujoco.mj_forward(model, data)
    steps = int(round(control_dt / model.opt.timestep))
    simulated_q = np.zeros_like(target_q_seq)
    for i, tq in enumerate(target_q_seq):
        for k, act_id in enumerate(act_ids):
            data.ctrl[act_id] = tq[k]
        for _ in range(steps):
            mujoco.mj_step(model, data)
        for k in range(len(act_ids)):
            simulated_q[i, k] = data.qpos[k]
    return simulated_q


def load_datasets(csv_paths, n_joints):
    datasets = []
    for path in csv_paths:
        df = pd.read_csv(path)
        target_q = df[[f"target_q{i+1}" for i in range(n_joints)]].to_numpy()
        actual_q = df[[f"actual_q{i+1}" for i in range(n_joints)]].to_numpy()
        datasets.append({"target_q": target_q, "actual_q": actual_q})
    return datasets


def objective_for_joint(params, joint_idx, current_kp, current_kv, model, data, act_ids, datasets, control_dt):
    kp_trial, kv_trial = current_kp.copy(), current_kv.copy()
    kp_trial[joint_idx], kv_trial[joint_idx] = params
    set_servo_gains(model, act_ids, kp_trial, kv_trial)

    total_sq_error, total_n = 0.0, 0
    for ds in datasets:
        sim_q = simulate_fast(model, data, act_ids, ds["target_q"], control_dt, ds["actual_q"][0])
        err = sim_q[:, joint_idx] - ds["actual_q"][:, joint_idx]
        total_sq_error += np.sum(err ** 2)
        total_n += len(err)
    return np.sqrt(total_sq_error / total_n)


def main():
    joint_names = common.get_joint_names(config.URDF_PATH)
    n_joints = len(joint_names)
    print(f"[0/3] URDF'den okunan eklemler ({n_joints} adet): {joint_names}\n")

    print(f"[1/3] {config.DATA_DIR} klasoru taraniyor (servo kazanci icin uygun veri)...")
    required_cols = common.make_required_columns_servo(n_joints)
    all_valid = common.discover_csv_files(required_cols)
    reliable = common.filter_by_tracking_quality(all_valid, n_joints)
    print(f"      -> {len(reliable)} guvenilir dosya kullanilacak.\n")

    datasets = load_datasets(reliable, n_joints)
    control_dt = 0.01

    mjcf_path = config.MJCF_WITH_FRICTION
    print(f"[2/3] Model yukleniyor: {mjcf_path}")
    model = mujoco.MjModel.from_xml_path(mjcf_path)
    data = mujoco.MjData(model)

    # HER EKLEM ICIN aktuator ID'sini ISIM DESENI VARSAYMADAN, MuJoCo'nun
    # kendi baglanti yapisindan (trnid) bul.
    act_ids = []
    for jn in joint_names:
        act_id = common.find_position_actuator_for_joint(model, jn)
        if act_id is None:
            raise RuntimeError(f"'{jn}' eklemini tahrik eden bir position aktuatoru bulunamadi!")
        act_ids.append(act_id)

    current_kp = np.zeros(n_joints)
    current_kv = np.zeros(n_joints)
    for idx, act_id in enumerate(act_ids):
        current_kp[idx] = model.actuator_gainprm[act_id, 0]
        current_kv[idx] = -model.actuator_biasprm[act_id, 2]

    print("\n[3/3] Koordinat inisi optimizasyonu (2 tur):")
    N_PASSES = 2
    for pass_num in range(N_PASSES):
        print(f"  === Tur {pass_num+1}/{N_PASSES} ===")
        for j in range(n_joints):
            # GUVENLIK: Eger baslangic kazanci 0 ise (URDF/MJCF'de kazanc tanimlanmamis
            # veya sifir birakilmissa), 0*0.2=0 ve 0*5.0=0 olur - alt ve ust sinir AYNI
            # (ikisi de 0) olur, L-BFGS-B bu daralmis (sifir genislikli) aralikta
            # hareket edemez ve optimizasyon SIKISIR. Bunu onlemek icin, hesaplanan
            # sinirlarin ALTINDA kalmayacagi guvenli birer minimum/maksimum tanimliyoruz.
            kp_bounds = (max(0.1, current_kp[j] * 0.2), max(5.0, current_kp[j] * 5.0))
            kv_bounds = (max(0.1, current_kv[j] * 0.2), max(5.0, current_kv[j] * 5.0))
            result = minimize(
                objective_for_joint, [current_kp[j], current_kv[j]],
                args=(j, current_kp, current_kv, model, data, act_ids, datasets, control_dt),
                method="L-BFGS-B", bounds=[kp_bounds, kv_bounds],
                options={"maxiter": 25, "eps": 1.0},
            )
            current_kp[j], current_kv[j] = result.x
            print(f"    {joint_names[j]}: kp={current_kp[j]:.1f} kv={current_kv[j]:.1f} "
                  f"(RMS: {result.fun:.5f} rad)")

    gains = {joint_names[j]: {"kp": float(current_kp[j]), "kv": float(current_kv[j])} for j in range(n_joints)}
    with open(config.FITTED_SERVO_GAINS_JSON, "w") as f:
        json.dump(gains, f, indent=2)
    print(f"\n[bilgi] Kaydedildi: {config.FITTED_SERVO_GAINS_JSON}")
    print("[bilgi] Sonraki adim: python scripts/05_apply_servo_gains_to_mjcf.py")


if __name__ == "__main__":
    main()
