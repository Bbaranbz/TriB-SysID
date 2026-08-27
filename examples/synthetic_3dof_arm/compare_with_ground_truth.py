"""
compare_with_ground_truth.py - Pipeline Ciktilarini Bilinen Degerlerle Karsilastir
======================================================================================

Bu script, pipeline'i bu ornek uzerinde calistirdiktan sonra, bulunan
Fv/Fc/kp/kv degerlerinin GROUND_TRUTH (bilinen, veriyi uretirken
kullanilan gercek) degerlerle ne kadar ORTUSTUGUNU gosterir.

KULLANIM:
  1. Bu klasordeki dosyalari ana proje yapisina yerlestirin:
       robot.urdf          -> models/robot.urdf
       model_ground_truth.xml -> models/model.xml
       trajectory_log.csv  -> data/raw/trajectory_log.csv
  2. Ana dizinden pipeline'i calistirin: python run_pipeline.py
  3. Bu scripti calistirin: python examples/synthetic_3dof_arm/compare_with_ground_truth.py

NOT: fitted_servo_gains.json OPSIYONELDIR. optional_mujoco_servo_module hic
calistirilmamissa, script sadece surtunme karsilastirmasini yapar, servo
kazanci bolumunu atlar - hata vermez.
"""

import json
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))


def main():
    with open(os.path.join(SCRIPT_DIR, "ground_truth.json")) as f:
        ground_truth = json.load(f)

    friction_path = os.path.join(PROJECT_ROOT, "outputs", "fitted_friction.json")
    gains_path = os.path.join(PROJECT_ROOT, "outputs", "fitted_servo_gains.json")

    if not os.path.exists(friction_path):
        print("[HATA] outputs/fitted_friction.json bulunamadi.")
        print("       Once ana dizinden 'python run_pipeline.py' calistirin.")
        return

    with open(friction_path) as f:
        fitted_friction = json.load(f)

    # Servo kazanci OPSIYONEL - optional_mujoco_servo_module hic calistirilmamis
    # olabilir, bu durumda sadece surtunme karsilastirmasi yapilir.
    fitted_gains = {}
    if os.path.exists(gains_path):
        with open(gains_path) as f:
            fitted_gains = json.load(f)
    else:
        print("[bilgi] fitted_servo_gains.json bulunamadi - servo kazanci karsilastirmasi ATLANACAK.")
        print("        (optional_mujoco_servo_module hic calistirilmamis olabilir, bu normal.)")

    print("\n=== SURTUNME KARSILASTIRMASI (Fv, Fc) ===")
    print(f"{'eklem':<10} {'gercek Fv':>10} {'bulunan Fv':>11} {'hata%':>7}   "
          f"{'gercek Fc':>10} {'bulunan Fc':>11} {'hata%':>7}")
    for joint in ground_truth:
        gt = ground_truth[joint]
        f_ = fitted_friction.get(joint, {})
        fv_err = 100 * abs(f_.get("Fv", 0) - gt["Fv"]) / gt["Fv"]
        fc_err = 100 * abs(f_.get("Fc", 0) - gt["Fc"]) / gt["Fc"]
        print(f"{joint:<10} {gt['Fv']:>10.3f} {f_.get('Fv', float('nan')):>11.3f} {fv_err:>6.0f}%   "
              f"{gt['Fc']:>10.3f} {f_.get('Fc', float('nan')):>11.3f} {fc_err:>6.0f}%")

    if fitted_gains:
        print("\n=== SERVO KAZANCI KARSILASTIRMASI (kp, kv) ===")
        print(f"{'eklem':<10} {'gercek kp':>10} {'bulunan kp':>11} {'hata%':>7}   "
              f"{'gercek kv':>10} {'bulunan kv':>11} {'hata%':>7}")
        for joint in ground_truth:
            gt = ground_truth[joint]
            g_ = fitted_gains.get(joint, {})
            kp_err = 100 * abs(g_.get("kp", 0) - gt["kp"]) / gt["kp"]
            kv_err = 100 * abs(g_.get("kv", 0) - gt["kv"]) / gt["kv"]
            print(f"{joint:<10} {gt['kp']:>10.1f} {g_.get('kp', float('nan')):>11.1f} {kp_err:>6.0f}%   "
                  f"{gt['kv']:>10.1f} {g_.get('kv', float('nan')):>11.1f} {kv_err:>6.0f}%")
    else:
        print("\n[bilgi] Servo kazanci verisi yok, bu bolum atlandi.")

    print("\n[bilgi] BEKLENEN SONUC:")
    print("  - Servo kazanclari (kp/kv): TAM ortusme beklenir (%0-1 hata) cunku")
    print("    bu, dogrudan kapali-dongu davranis eslestirmesi ile bulunuyor.")
    print("  - Surtunme (Fv/Fc): Genis hiz araligina sahip eklemlerde (bu ornekte")
    print("    'waist') neredeyse TAM ortusme beklenir. Dar hiz araligina sahip")
    print("    eklemlerde Fv sifira dusebilir (Fv/sign(qd) ayristirilamama -")
    print("    coklu dogrusal baglanti sorunu) - bu, GERCEK Lebai robot verisinde")
    print("    de gozlemlenen, veri kisitina bagli BEKLENEN bir davranistir,")
    print("    tanimlama algoritmasinda bir hata degildir. Fc genelde daha")
    print("    guvenilir sekilde (%10-15 hata payinda) tahmin edilir.")


if __name__ == "__main__":
    main()