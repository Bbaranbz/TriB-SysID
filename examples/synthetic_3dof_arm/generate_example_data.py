"""
generate_example_data.py - Bilinen (Ground Truth) Parametrelerle Sentetik Veri Uretimi
===========================================================================================

AMAC: Pipeline'in DOGRU calistigini kanitlamak icin, sürtünme (Fv, Fc) ve
servo kazanclarinin (kp, kv) ONCEDEN BILINDIGI bir MJCF modeli kurup,
gercek bir KAPALI-DONGU MuJoCo simulasyonu calistiriyoruz. Cikan veri,
gercek bir robottan gelen veriyle AYNI formatta (target_q, actual_q,
actual_qd, actual_tau) kaydediliyor.

Pipeline bu veriyi isleyip Fv, Fc, kp, kv tahmin ettiginde, sonuc
ASAGIDAKI GROUND_TRUTH degerleriyle (kucuk sayisal hatalar disinda)
ORTUSMELI. Ortusmuyorsa, pipeline'da bir hata var demektir.

NEDEN GERCEK SIMULASYON (analitik formul degil)?
Boylece hem surtunme hem servo kazanci ayni, TUTARLI fizik motorundan
uretiliyor - actual_tau = MuJoCo'nun actuator_force'u, ki bu tam olarak
pipeline'in "actual_tau" olarak bekledigi buyuklukle ORTUSUYOR (motor
torku = rijit govde dinamigi + surtunme, PD kontrolcunun dengeledigi tam
miktar).
"""

import numpy as np
import pandas as pd
import mujoco

GROUND_TRUTH = {
    "waist":    {"Fv": 0.45, "Fc": 1.20, "kp": 300, "kv": 30},
    "shoulder": {"Fv": 0.30, "Fc": 0.85, "kp": 220, "kv": 22},
    "wrist":    {"Fv": 0.12, "Fc": 0.35, "kp": 90,  "kv": 9},
}
JOINT_NAMES = ["waist", "shoulder", "wrist"]


def generate_multisine_trajectory(t, amplitudes, frequencies, phase_offsets):
    """Her eklem icin genis bir HIZ araligi saglayan coklu-sinuzoidal
    (multi-sine) uyarma trajectorisi - dusuk VE yuksek hiz karisik
    olsun diye BIRDEN FAZLA frekans BILESENI toplaniyor (Fourier serisi
    mantigi). Bu, projenin README'sindeki "genis hiz cesitliligi sarti"nin
    dogrudan uygulamasidir."""
    n_joints = len(amplitudes)
    N = len(t)
    q = np.zeros((N, n_joints))
    qd = np.zeros((N, n_joints))
    for j in range(n_joints):
        for k, (amp, freq, phase) in enumerate(zip(amplitudes[j], frequencies[j], phase_offsets[j])):
            q[:, j] += amp * np.sin(2 * np.pi * freq * t + phase)
            qd[:, j] += amp * 2 * np.pi * freq * np.cos(2 * np.pi * freq * t + phase)
    return q, qd


def main():
    model = mujoco.MjModel.from_xml_path("model_ground_truth.xml")
    data = mujoco.MjData(model)

    fs = 100.0
    T = 40.0
    t = np.arange(0, T, 1.0 / fs)
    N = len(t)
    # Her eklem icin 2-3 frekans bileseni: genis genlik + genis hiz araligi
    # (dusuk hiz VE yuksek hiz ayni veri setinde bulunsun diye)
    amplitudes = [
        [0.6, 0.25],   # waist
        [0.5, 0.20],  # shoulder
        [0.8, 0.30],  # wrist
    ]
    frequencies = [
        [0.08, 0.25],  # waist: yavas + hizli bileşen
        [0.10, 0.28],  # shoulder
        [0.12, 0.32],  # wrist
    ]
    phase_offsets = [
        [0.0, 0.7],
        [1.2, 0.3],
        [0.5, 1.9],
    ]

    target_q, _ = generate_multisine_trajectory(t, amplitudes, frequencies, phase_offsets)

    # Baslangic pozisyonunu ilk hedefe esitle (ani sicrama olmasin)
    data.qpos[:3] = target_q[0]
    mujoco.mj_forward(model, data)

    control_dt = 1.0 / fs
    steps_per_control = int(round(control_dt / model.opt.timestep))

    log_q = np.zeros((N, 3))
    log_qd = np.zeros((N, 3))
    log_tau = np.zeros((N, 3))

    Fv_array = np.array([GROUND_TRUTH[n]["Fv"] for n in JOINT_NAMES])
    Fc_array = np.array([GROUND_TRUTH[n]["Fc"] for n in JOINT_NAMES])

    for i in range(N):
        data.ctrl[:3] = target_q[i]
        for _ in range(steps_per_control):
            # KENDI IDEAL SURTUNME MODELIMIZI (Fv*qd + Fc*sign(qd)) doğrudan
            # bir DIS KUVVET (qfrc_applied) olarak enjekte ediyoruz - MuJoCo'nun
            # kendi 'frictionloss' mekanizmasini KULLANMIYORUZ, cunku o
            # (constraint-solver tabanli) ideal Coulomb modelinden farkli
            # davraniyor. Boylece uretilen veri, tanimlama algoritmamizin
            # VARSAYDIGI modelle TAM TUTARLI olur - "temiz" bir dogrulama saglar.
            qd_current = data.qvel[:3]
            friction_force = -(Fv_array * qd_current + Fc_array * np.sign(qd_current))
            data.qfrc_applied[:3] = friction_force
            mujoco.mj_step(model, data)
        log_q[i] = data.qpos[:3].copy()
        log_qd[i] = data.qvel[:3].copy()
        # actuator_force: PD kontrolcunun urettigi GERCEK motor torku -
        # rijit govde dinamigi + bizim enjekte ettigimiz surtunmeyi
        # dengelemek icin ne kadar tork urettigi.
        log_tau[i] = data.actuator_force[:3].copy()

    # Kucuk olcum gurultusu ekle (gercekci olsun diye)
    rng = np.random.default_rng(42)
    log_tau_noisy = log_tau + rng.normal(0, 0.01, log_tau.shape)
    log_qd_noisy = log_qd + rng.normal(0, 0.002, log_qd.shape)

    df = pd.DataFrame({"time_s": t})
    for j, name in enumerate(JOINT_NAMES):
        df[f"target_q{j+1}"] = target_q[:, j]
        df[f"actual_q{j+1}"] = log_q[:, j]
        df[f"actual_qd{j+1}"] = log_qd_noisy[:, j]
        df[f"actual_tau{j+1}"] = log_tau_noisy[:, j]

    df.to_csv("trajectory_log.csv", index=False)

    own_tracking_error = np.sqrt(np.mean((target_q - log_q) ** 2))
    print(f"[bilgi] Veri uretildi: {df.shape[0]} satir, {fs} Hz, {T} saniye")
    print(f"[bilgi] Robotun kendi target/actual takip hatasi: {own_tracking_error:.4f} rad "
          f"(0.05 esiginin {'ALTINDA - servo kazanci tanimlamaya uygun' if own_tracking_error < 0.05 else 'USTUNDE - dikkat!'})")
    print(f"[bilgi] Hiz araligi (rad/s):")
    for j, name in enumerate(JOINT_NAMES):
        print(f"    {name}: [{log_qd[:,j].min():.3f}, {log_qd[:,j].max():.3f}]")

    import json
    with open("ground_truth.json", "w") as f:
        json.dump(GROUND_TRUTH, f, indent=2)
    print("\n[bilgi] Ground truth (gercek/beklenen) degerler ground_truth.json'a kaydedildi.")
    print("[bilgi] Pipeline'i calistirdiktan sonra outputs/fitted_friction.json ve")
    print("        outputs/fitted_servo_gains.json dosyalarini bu degerlerle karsilastirin.")


if __name__ == "__main__":
    main()
