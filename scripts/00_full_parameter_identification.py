"""
00_full_parameter_identification.py - TUM Dinamik Parametreleri Tanimla
============================================================================

Bu script, kutle/atalet parametrelerinin ARTIK BILINEN kabul EDILMEDIGI,
tum dinamik parametrelerin (kutle, kutle merkezi, atalet tensoru, VE
surtunme) BIRLIKTE regresor yontemiyle tahmin edildigi GENEL modu
calistirir.

NE ZAMAN KULLANILIR: config.IDENTIFY_FULL_INERTIAL_PARAMS = True
ise, veya URDF'deki kutle/atalet degerlerine GUVENMIYORSANIZ / bunlari
da GERCEK ROBOT verisiyle DOGRULAMAK/DUZELTMEK istiyorsaniz.

** ONEMLI ON KOSUL - LUTFEN OKUYUN **
Sadece sürtünme tanimlamak icin kullandigimiz basit (dar aralikli,
küçük genlikli) trajectory'ler bu modda YETERSIZ KALIR. Tam parametre
tanimlama, robotun COK GENIS bir konfigurasyon/hiz/ivme uzayini
tarayan, ozel olarak optimize edilmis ("persistently exciting")
trajectoriler gerektirir - aksi halde asagidaki "rank" sayisi cok
dusuk cikar ve tahminler guvenilmez olur.

CIKTI: outputs/base_parameters_report.json - hangi parametre
kombinasyonlarinin tanimlanabilir oldugunu, hangilerinin OLMADIGINI
acikca raporlar. TUM 60+12 parametre icin "sonuc" UYDURULMAZ - sadece
matematiksel olarak gozlemlenebilir olanlar icin sayisal deger verilir.
"""

import json
import os
import sys

import numpy as np

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

    print("[2/4] TAM regresor matrisi olusturuluyor "
          "(her zaman orneginde atalet+surtunme regresoru hesaplaniyor)...")
    print("      Bu adim, sadece surtunme tanimlamadan DAHA YAVAS calisir "
          "(her ornek icin 60+12 sutunlu bir matris hesaplaniyor).")
    Y_full, tau_full, model, nv = common.build_stacked_regressor(csv_paths, config.URDF_PATH)
    n_params_total = Y_full.shape[1]
    print(f"      -> Regresor boyutu: {Y_full.shape} "
          f"({Y_full.shape[0]} denklem, {n_params_total} ham parametre)\n")

    print("[3/4] QR ayristirmasi ile tanimlanabilir (base) parametreler bulunuyor...")
    independent_idx, rank = common.compute_base_parameters(Y_full)
    param_names = common.get_parameter_names(nv, joint_names=joint_names)
    identifiable_names = [param_names[i] for i in independent_idx]

    print(f"      -> Toplam {n_params_total} ham parametreden "
          f"{rank} tanesi BAGIMSIZ (tanimlanabilir).")
    print(f"      -> Veri kalitesi orani: %{100*rank/n_params_total:.0f} "
          f"(bu oran ne kadar YUKSEKSE trajectory'niz o kadar zengin demektir)")

    if rank < n_params_total * 0.5:
        print("\n      [UYARI] Tanimlanabilir parametre orani COK DUSUK (<%50). "
              "Bu, mevcut trajectory verinizin tam parametre tanimlama icin "
              "YETERSIZ oldugunu gosteriyor. Sonuclari DIKKATLE yorumlayin, "
              "mumkunse cok daha zengin/genis bir uyarma trajectorisi ile "
              "yeniden veri toplayin.")

    print("\n[4/4] Bagimsiz parametreler icin en kucuk kareler cozuluyor...")
    phi_reduced = common.fit_base_parameters(Y_full, tau_full, independent_idx)

    # Tahmin kalitesini kontrol et (fit sonrasi residual)
    Y_reduced = Y_full[:, independent_idx]
    tau_pred = Y_reduced @ phi_reduced
    rms_fit = np.sqrt(np.mean((tau_full - tau_pred) ** 2))
    rms_baseline = np.sqrt(np.mean(tau_full ** 2))
    print(f"      -> Fit sonrasi RMS: {rms_fit:.4f} Nm "
          f"(referans - tork buyuklugu: {rms_baseline:.4f} Nm)")

    report = {
        "n_ham_parametre": n_params_total,
        "n_tanimlanabilir_parametre": rank,
        "tanimlanabilirlik_orani_yuzde": round(100 * rank / n_params_total, 1),
        "fit_rms_Nm": float(rms_fit),
        "tanimlanabilir_parametreler": {
            name: float(val) for name, val in zip(identifiable_names, phi_reduced)
        },
        "TANIMLANAMAYAN_parametreler_NOT": (
            "Bu listede olmayan parametreler (orn. bazi linklerin ayri kutle "
            "merkezi bilesenleri) mevcut veriyle TEK BASINA gozlemlenemedi - "
            "bunlar icin sayisal bir deger UYDURULMADI. URDF'deki mevcut "
            "(uretici) degerleri korumanizi oneririz."
        ),
    }

    # -----------------------------------------------------------------
    # FIZIKSEL TUTARLILIK KONTROLU
    # "Bagimsiz" (rank'e giren) bir parametre, illa GUVENILIR demek
    # degildir - eger o parametrenin veri icindeki etkisi kucukse,
    # gurultu tahmini anlamsiz buyutebilir (kotu kosullanma). Bunu
    # yakalamanin basit bir yolu: atalet (Ixx, Iyy, Izz benzeri) ve
    # kutle (m) parametrelerinin FIZIKSEL OLARAK GECERLI (pozitif)
    # araliklarda olup olmadigini kontrol etmek.
    # -----------------------------------------------------------------
    IMPLAUSIBLE_THRESHOLD = 50.0  # Nm.s^2 mertebesinde bir atalet, kucuk bir robot kolu icin imkansiz buyuklukte
    suspicious = []
    for name, val in report["tanimlanabilir_parametreler"].items():
        is_diagonal_inertia = any(name.endswith(s) for s in ["_Ixx", "_Iyy", "_Izz"])
        is_mass = name.endswith("_m")
        if (is_diagonal_inertia and val < 0) or (is_mass and val < 0):
            suspicious.append((name, val, "NEGATIF (fiziksel olarak imkansiz)"))
        elif abs(val) > IMPLAUSIBLE_THRESHOLD:
            suspicious.append((name, val, f"MAKUL OLMAYAN BUYUKLUK (>{IMPLAUSIBLE_THRESHOLD})"))

    report["fiziksel_tutarlilik_uyarilari"] = [
        {"parametre": n, "deger": v, "sorun": s} for n, v, s in suspicious
    ]

    if suspicious:
        print(f"\n      [FIZIKSEL TUTARLILIK UYARISI] {len(suspicious)} parametre "
              f"fiziksel olarak MAKUL DEGIL cikti:")
        for n, v, s in suspicious[:5]:
            print(f"        - {n} = {v:.2f}  ({s})")
        if len(suspicious) > 5:
            print(f"        ... ve {len(suspicious)-5} tane daha (tam liste JSON raporunda)")
        print("      BU SONUCLARI MODELE ISLEMEYIN. Bu, mevcut trajectory'nin tam")
        print("      atalet tanimlama icin YETERSIZ oldugunu gosteriyor - kutle/atalet")
        print("      icin URDF'deki (uretici) degerleri korumaya devam edin.")

    with open(config.BASE_PARAMS_REPORT_JSON, "w") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    print(f"\n[bilgi] Rapor kaydedildi: {config.BASE_PARAMS_REPORT_JSON}")
    print("[bilgi] NOT: Bu script URDF/MJCF dosyalarini OTOMATIK GUNCELLEMEZ -")
    print("       tanimlanabilirlik orani dusukse (yukaridaki uyariyi kontrol edin)")
    print("       bulunan degerleri modele islemeden once MUTLAKA fiziksel")
    print("       tutarliligini (orn. atalet ozdegerlerinin pozitif olmasi) kontrol edin.")


if __name__ == "__main__":
    main()
