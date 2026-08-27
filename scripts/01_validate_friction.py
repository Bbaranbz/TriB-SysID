"""
01_validate_friction.py - Surtunme (Fv, Fc) Tanimlama ve Dogrulama
======================================================================

KULLANIM: python scripts/01_validate_friction.py

Bu script HERHANGI BIR ROBOT icin calisir - eklem sayisi ve isimleri
models/robot.urdf dosyasindan OTOMATIK okunur, hicbir yerde sabit
"6 eklem" veya "joint_1" gibi bir varsayim yoktur.

data/raw/ klasorundeki TUM gecerli CSV dosyalari OTOMATIK bulunur.
Beklenen sutun formati: actual_q1..N, actual_qd1..N, actual_tau1..N
(N = robotunuzun eklem sayisi, URDF'den otomatik belirlenir).
"""

import json
import os
import sys

from src import config
from src import common


def main():
    joint_names = common.get_joint_names(config.URDF_PATH)
    n_joints = len(joint_names)
    print(f"[0/4] URDF'den okunan eklemler ({n_joints} adet): {joint_names}\n")

    required_cols = common.make_required_columns_friction(n_joints)

    print(f"[1/4] {config.DATA_DIR} klasoru taraniyor...")
    csv_paths = common.discover_csv_files(required_cols)
    print(f"      -> {len(csv_paths)} gecerli dosya bulundu.\n")

    print("[2/4] RNEA ile residual hesaplaniyor (her dosya ayri isleniyor)...")
    Q, QD, QDD, Tau, residual, file_lengths = common.compute_residual_multi(
        csv_paths, config.URDF_PATH
    )
    print(f"      -> Toplam {len(Q)} ornek, dosya uzunluklari: {file_lengths}\n")

    print("[3/4] Dogrulama testleri:")
    print("  --- A) Korelasyon testi (surtunme mi, model hatasi mi?) ---")
    for j in range(1, n_joints + 1):
        corr_q, corr_signqd = common.correlation_check(Q, QD, residual, j)
        yorum = "GUCLU surtunme sinyali" if corr_signqd > corr_q else "DIKKAT: pozisyona bagli olabilir"
        print(f"    {joint_names[j-1]}: corr(residual,pozisyon)={corr_q:+.3f}  "
              f"corr(residual,hiz_yonu)={corr_signqd:+.3f}  -> {yorum}")

    print("\n  --- B) Dosya-farkindali train/test dogrulamasi ---")
    for j in range(1, n_joints + 1):
        r = common.train_test_validate_multi(QD, residual, file_lengths, j)
        print(f"    {joint_names[j-1]}: Fv={r['Fv']:.3f} Fc={r['Fc']:.3f}  |  "
              f"test RMS={r['rms_test_with_fit']:.3f} Nm "
              f"(fitsiz {r['rms_test_without_fit']:.3f}, iyilesme %{r['improvement_percent']:.0f})")

    print("\n[4/4] Nihai parametreler (TUM veriyle fit) kaydediliyor...")
    final_params = {}
    for j in range(1, n_joints + 1):
        Fv, Fc, offset = common.fit_friction(QD[:, j - 1], residual[:, j - 1])
        joint_name = joint_names[j - 1]
        final_params[joint_name] = {"Fv": float(Fv), "Fc": float(Fc), "Foffset": float(offset)}
        print(f"    {joint_name}: Fv={Fv:.4f} Nm.s/rad, Fc={Fc:.4f} Nm")

    with open(config.FITTED_FRICTION_JSON, "w") as f:
        json.dump(final_params, f, indent=2)
    print(f"\n[bilgi] Kaydedildi: {config.FITTED_FRICTION_JSON}")
    print("[bilgi] Sonraki adim: python scripts/02_apply_friction_to_urdf.py")


if __name__ == "__main__":
    main()
