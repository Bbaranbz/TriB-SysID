"""
common.py - Ortak Fonksiyonlar
==================================

Tum scriptlerin paylastigi temel islevler burada toplanmistir:
  - Veri klasorunden GECERLI CSV dosyalarini OTOMATIK bulma
  - Filtreleme (Butterworth alcak-gecirgen)
  - RNEA ile residual (surtunme sinyali) hesaplama
  - Sürtünme fit etme (en kucuk kareler)

Yeni bir veri dosyasi eklemek istediginizde bu dosyayi DEGISTIRMENIZE
gerek yoktur - sadece CSV'yi data/raw/ klasorune atmaniz yeterlidir,
discover_csv_files() onu otomatik bulacaktir.
"""

import glob
import os

import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt
from scipy.optimize import lsq_linear

from . import config


def get_joint_names(urdf_path):
    """
    URDF'deki HAREKETLI eklemlerin isimlerini, tanimlandiklari SIRAYLA
    dondurur. Bu fonksiyon, projenin "herhangi bir robotla calisabilme"
    ozelliginin temelidir - hicbir yerde "joint_1", "6 eklem" gibi
    SABIT bir varsayim YAPILMAZ, her sey buradan TURETILIR.

    Pinocchio, modelin 0. indeksine her zaman "universe" (sabit dunya
    çerçevesi) koyar - bu gercek bir eklem olmadigi icin atlanir.
    """
    import pinocchio as pin
    model = pin.buildModelFromUrdf(urdf_path)
    return list(model.names[1:])  # ilk eleman "universe", atla


def get_n_joints(urdf_path):
    return len(get_joint_names(urdf_path))


def make_required_columns_friction(n_joints):
    """CSV formatinda beklenen sutunlar - SAYISAL indeksleme kullanir
    (actual_q1, actual_q2, ...) cunku bu, robotun eklem ISIMLERINDEN
    BAGIMSIZ, evrensel bir veri kaydi kuralidir (bkz. README - veri
    formati bolumu)."""
    return (
        [f"actual_q{i+1}" for i in range(n_joints)]
        + [f"actual_qd{i+1}" for i in range(n_joints)]
        + [f"actual_tau{i+1}" for i in range(n_joints)]
    )


def make_required_columns_servo(n_joints):
    return (
        [f"target_q{i+1}" for i in range(n_joints)]
        + [f"actual_q{i+1}" for i in range(n_joints)]
    )


# Geriye-uyumluluk icin: eger bir script N_JOINTS bilmeden sadece
# "gerekli sutun ORNEGI" gormek isterse (ornegin hata mesaji icin),
# bu sabitler 6 eklemli bir robot ORNEGINI gosterir - ama gercek
# calisma zamaninda TUM scriptler make_required_columns_*(n_joints)
# fonksiyonlarini gercek n_joints ile CAGIRIR, bu sabitleri kullanmaz.
REQUIRED_COLUMNS_FRICTION = make_required_columns_friction(6)
REQUIRED_COLUMNS_SERVO = make_required_columns_servo(6)


def discover_csv_files(required_columns, verbose=True):
    """
    data/raw/ klasorundeki TUM .csv dosyalarini tarar, GEREKLI SUTUNLARA
    sahip olanlari dondurur. Sahip olmayanlar (ornegin sadece "target_q"
    iceren, "actual_q" icermeyen bir trajectory tasarim dosyasi gibi)
    OTOMATIK OLARAK ATLANIR, hata vermez - boylece data/raw/ klasorune
    farkli turde dosyalar da (yanlislikla) konsa bile script cokmez.

    Bu fonksiyon, projenin "yeni veri = sadece dosya at" felsefesinin
    kalbidir: script kodu hicbir zaman dosya adi bilmez, her calistirmada
    klasoru YENIDEN tarar.
    """
    all_csvs = sorted(glob.glob(os.path.join(config.DATA_DIR, "*.csv")))

    if not all_csvs:
        raise RuntimeError(
            f"HATA: {config.DATA_DIR} klasorunde hic CSV dosyasi bulunamadi. "
            f"Robottan topladiginiz trajectory_log*.csv dosyalarini bu klasore atin."
        )

    valid_files = []
    for path in all_csvs:
        try:
            header = pd.read_csv(path, nrows=0).columns.tolist()
        except Exception as e:
            if verbose:
                print(f"  [ATLANDI] {os.path.basename(path)}: okunamadi ({e})")
            continue

        missing = [c for c in required_columns if c not in header]
        if missing:
            if verbose:
                print(f"  [ATLANDI] {os.path.basename(path)}: gerekli sutunlar eksik "
                      f"(ornek eksik: {missing[0]}) - bu muhtemelen bir trajectory "
                      f"TASARIM dosyasi, gercek olcum kaydi degil.")
            continue

        valid_files.append(path)
        if verbose:
            print(f"  [ok] {os.path.basename(path)}")

    if not valid_files:
        raise RuntimeError(
            "HATA: Gerekli sutunlara sahip HICBIR CSV dosyasi bulunamadi. "
            f"Gerekli sutunlar: {required_columns[:3]}..."
        )

    return valid_files


def filter_by_tracking_quality(csv_paths, n_joints, threshold_rad=None, verbose=True):
    """
    Servo kazanci (kp/kv) tanimlama gibi target-vs-actual karsilastirmasi
    gerektiren islemler icin, robotun KENDI target/actual takip hatasi
    BUYUK olan dosyalari otomatik dislar (bkz. proje gecmisi: yuksek
    hizli/genis genlikli trajectory'lerde robotun kendisi bile hedefi
    tutturamadigi durumlar goruldu - bu tur veri kp/kv tanimlamayi
    kirletir).
    """
    threshold = threshold_rad or config.OWN_TRACKING_ERROR_THRESHOLD_RAD
    accepted = []
    for path in csv_paths:
        df = pd.read_csv(path)
        target_q = df[[f"target_q{i+1}" for i in range(n_joints)]].to_numpy()
        actual_q = df[[f"actual_q{i+1}" for i in range(n_joints)]].to_numpy()
        err = np.sqrt(np.mean((target_q - actual_q) ** 2))

        if err > threshold:
            if verbose:
                print(f"  [DISLANDI] {os.path.basename(path)}: robotun kendi takip "
                      f"hatasi {err:.3f} rad (esik: {threshold} rad) - GUVENILMEZ.")
            continue

        if verbose:
            print(f"  [ok] {os.path.basename(path)}: takip hatasi {err:.4f} rad")
        accepted.append(path)

    if not accepted:
        raise RuntimeError("Hicbir dosya guvenilirlik esigini gecemedi.")

    return accepted


def lowpass_filter(x, cutoff_hz, fs_hz, order=None):
    """4. derece (varsayilan) Butterworth alcak-gecirgen filtre."""
    if order is None:
        order = config.FILTER_ORDER
    nyquist = 0.5 * fs_hz
    b, a = butter(order, cutoff_hz / nyquist, btype="low")
    return filtfilt(b, a, x, axis=0)


def estimate_fs(t):
    return 1.0 / np.median(np.diff(t))


def compute_residual(csv_path, model, data, cutoff_hz=None, cutoff_hz_accel=None):
    """Tek bir CSV dosyasi icin RNEA residual'ini hesaplar.
    (model, data): onceden yuklenmis pinocchio model/data nesneleri -
    her dosya icin yeniden yuklenmez, performans icin disaridan verilir.
    n_joints, model.nv'den OTOMATIK alinir - hicbir yerde sabit sayi yok.
    """
    import pinocchio as pin

    cutoff_hz = cutoff_hz or config.LOWPASS_CUTOFF_HZ
    cutoff_hz_accel = cutoff_hz_accel or config.LOWPASS_CUTOFF_HZ_ACCEL
    n_joints = model.nv

    df = pd.read_csv(csv_path)
    t = df["time_s"].to_numpy() if "time_s" in df.columns else np.arange(len(df)) * 0.01
    fs = estimate_fs(t)

    Q = df[[f"actual_q{i+1}" for i in range(n_joints)]].to_numpy()
    QD_given = df[[f"actual_qd{i+1}" for i in range(n_joints)]].to_numpy()
    Tau = df[[f"actual_tau{i+1}" for i in range(n_joints)]].to_numpy()

    Q_f = lowpass_filter(Q, cutoff_hz, fs)
    Tau_f = lowpass_filter(Tau, cutoff_hz, fs)
    QD_f = lowpass_filter(QD_given, cutoff_hz, fs)
    QDD_f = lowpass_filter(np.gradient(QD_f, 1.0 / fs, axis=0), cutoff_hz_accel, fs)

    N = len(Q_f)
    Tau_model = np.zeros((N, n_joints))
    for i in range(N):
        Tau_model[i] = pin.rnea(model, data, Q_f[i], QD_f[i], QDD_f[i])

    residual = Tau_f - Tau_model
    return Q_f, QD_f, QDD_f, Tau_f, residual


def compute_residual_multi(csv_paths, urdf_path):
    """Birden fazla dosyayi ayri ayri isleyip (turev/filtre tutarliligi
    icin) sonuclari birlestirir. file_lengths, sonradan dosya-farkindali
    train/test split yapabilmek icin dondurulur."""
    import pinocchio as pin

    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()

    all_Q, all_QD, all_QDD, all_Tau, all_residual = [], [], [], [], []
    file_lengths = []

    for path in csv_paths:
        Q, QD, QDD, Tau, residual = compute_residual(path, model, data)
        all_Q.append(Q); all_QD.append(QD); all_QDD.append(QDD)
        all_Tau.append(Tau); all_residual.append(residual)
        file_lengths.append(len(Q))

    return (np.concatenate(all_Q), np.concatenate(all_QD), np.concatenate(all_QDD),
            np.concatenate(all_Tau), np.concatenate(all_residual), file_lengths)


def fit_friction(qd, residual, qd_threshold=0.01):
    """tau_friction = Fv*qd + Fc*sign(qd) + offset - fiziksel olarak
    Fv>=0, Fc>=0 kisitiyla (sinirli en kucuk kareler)."""
    mask = np.abs(qd) > qd_threshold
    qd_use, res_use = qd[mask], residual[mask]
    A = np.column_stack([qd_use, np.sign(qd_use), np.ones_like(qd_use)])
    result = lsq_linear(A, res_use, bounds=([0, 0, -np.inf], [np.inf, np.inf, np.inf]))
    return result.x  # Fv, Fc, offset


def train_test_validate_multi(QD, residual, file_lengths, joint_idx, train_fraction=0.7):
    """Her dosyayi KENDI icinde train/test bolup sonuclari birlestirir -
    boylece kisa bir dosyanin TAMAMEN teste dusup egitime hic girmemesi
    onlenir (bkz. proje gecmisi: bu hata ilk versiyonda yasanmisti)."""
    j = joint_idx - 1
    train_qd, train_res, test_qd, test_res = [], [], [], []
    start = 0
    for length in file_lengths:
        end = start + length
        split = start + int(length * train_fraction)
        train_qd.append(QD[start:split, j]); train_res.append(residual[start:split, j])
        test_qd.append(QD[split:end, j]); test_res.append(residual[split:end, j])
        start = end

    qd_tr, res_tr = np.concatenate(train_qd), np.concatenate(train_res)
    qd_te, res_te = np.concatenate(test_qd), np.concatenate(test_res)

    Fv, Fc, offset = fit_friction(qd_tr, res_tr)
    pred_te = Fv * qd_te + Fc * np.sign(qd_te) + offset
    rms_with = np.sqrt(np.mean((res_te - pred_te) ** 2))
    rms_without = np.sqrt(np.mean(res_te ** 2))
    improvement = (1 - rms_with / rms_without) * 100 if rms_without > 0 else 0

    return {"Fv": Fv, "Fc": Fc, "offset": offset,
            "rms_test_with_fit": rms_with, "rms_test_without_fit": rms_without,
            "improvement_percent": improvement}


def correlation_check(Q, QD, residual, joint_idx):
    j = joint_idx - 1
    corr_q = np.corrcoef(Q[:, j], residual[:, j])[0, 1]
    corr_signqd = np.corrcoef(np.sign(QD[:, j]), residual[:, j])[0, 1]
    return corr_q, corr_signqd


# =============================================================================
# MUJOCO AKTUATOR ESLESTIRME (herhangi bir isimlendirme semasi icin genel)
# =============================================================================

def find_position_actuator_for_joint(mj_model, joint_name):
    """
    'joint_1_position' gibi bir isim DESENI VARSAYMAK yerine, MuJoCo'nun
    kendi ic veri yapisini kullanarak aktuatoru BULUR.
    """
    try:
        import mujoco
    except ImportError:
        import warnings
        warnings.warn(
            "MuJoCo kutuphanesi bulunamadi - bu opsiyonel ozelligi kullanmak "
            "icin 'pip install mujoco' ile yukleyin.",
            stacklevel=2,
        )
        return None

    joint_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        return None

    for act_id in range(mj_model.nu):
        if mj_model.actuator_trntype[act_id] == 0 and mj_model.actuator_trnid[act_id, 0] == joint_id:
            return act_id
    return None


# =============================================================================
# TAM PARAMETRE TANIMLAMA (kutle, kutle merkezi, atalet + surtunme, HEP BIRLIKTE)
# =============================================================================
#
# Yukaridaki fonksiyonlar "residual yontemi" kullanir: kutle/atalet BILINEN
# kabul edilip, sadece surtunme (2 parametre/eklem) aranir. Bu bolum ise
# HICBIR SEYI bilinen kabul etmez - klasik "tam dinamik parametre
# tanimlama" (full rigid-body regressor identification) yontemidir.
#
# TEMEL FARK: tau = Y(q,qd,qdd) * phi
#   phi = [her link icin 10 parametre (m, m*cx, m*cy, m*cz, Ixx, Ixy,
#          Iyy, Ixz, Iyz, Izz), + her eklem icin Fv, Fc] birlesik vektoru
#   Y   = pinocchio'nun computeJointTorqueRegressor() ile REGRESOR
#         matrisi (kutle/atalet kismi) + surtunme regresoru (qd, sign(qd))
#         yan yana eklenmis hali
#
# NEDEN "BASE PARAMETERS" GEREKLI?
# 60 ham atalet parametresinin hepsi AYRI AYRI gozlemlenebilir degildir -
# bazilari sadece BELIRLI LINEER KOMBINASYONLARI halinde torka etki eder.
# Ornegin bir linkin kutlesi ile onun kutle merkezi konumu genelde CARPIM
# halinde gorunur, ayri ayri cozulemez. Bunu bulmak icin regresor
# matrisine SUTUN PIVOTLU QR AYRISTIRMASI uygulanir - matrisin bagimsiz
# (RANK'i belirleyen) sutunlari, "base parameters" (tanimlanabilir minimal
# kume) olarak alinir. Geri kalan parametreler tek basina anlamli degildir.

import pinocchio as pin


def build_full_regressor(model, data, q, qd, qdd):
    """Tek bir zaman ani icin: [atalet regresoru | surtunme regresoru]
    yan yana birlestirilmis TAM regresor matrisini dondurur.

    Atalet kismi: pin.computeJointTorqueRegressor() -> (nv x 10*nbodies)
    Surtunme kismi: her eklem j icin iki sutun [qd_j, sign(qd_j)],
    diger eklemlere karsilik gelen satirlarda SIFIR (cunku eklem j'nin
    surtunmesi sadece eklem j'nin kendi tork denklemine girer).
    """
    Y_inertial = pin.computeJointTorqueRegressor(model, data, q, qd, qdd)
    nv = model.nv

    Y_friction = np.zeros((nv, 2 * nv))
    for j in range(nv):
        Y_friction[j, 2 * j] = qd[j]
        Y_friction[j, 2 * j + 1] = np.sign(qd[j])

    return np.hstack([Y_inertial, Y_friction])


def build_stacked_regressor(csv_paths, urdf_path, cutoff_hz=None, cutoff_hz_accel=None):
    """Birden fazla dosyadan TUM zaman orneklerini isleyip, her biri icin
    tam regresoru hesaplayip DIKEY olarak (satir satir) yigar. Sonuc:
    Y_full: (N_ornek * nv) x (10*nbodies + 2*nv)
    tau_full: (N_ornek * nv,) - gercek olculen tork, aynı sirada.
    """
    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()
    nv = model.nv

    Y_rows = []
    tau_rows = []

    for path in csv_paths:
        Q, QD, QDD, Tau, _ = compute_residual(path, model, data, cutoff_hz, cutoff_hz_accel)
        for i in range(len(Q)):
            Y_i = build_full_regressor(model, data, Q[i], QD[i], QDD[i])
            Y_rows.append(Y_i)
            tau_rows.append(Tau[i])

    Y_full = np.vstack(Y_rows)
    tau_full = np.concatenate(tau_rows)
    return Y_full, tau_full, model, nv


def get_parameter_names(nv, joint_names=None):
    """Her sutunun HANGI fiziksel parametreye karsilik geldigini
    okunabilir isimlerle dondurur - QR sonrasi rapor icin.
    joint_names verilirse (URDF'den okunan gercek isimler), link
    numarasi yerine gercek eklem/link ismi kullanilir - okunabilirligi
    artirir ve rapor herhangi bir robotta anlamli kalir."""
    names = []
    for link in range(1, nv + 1):
        label = joint_names[link - 1] if joint_names else f"link{link}"
        for p in ["m", "m*cx", "m*cy", "m*cz", "Ixx", "Ixy", "Iyy", "Ixz", "Iyz", "Izz"]:
            names.append(f"{label}_{p}")
    for j in range(1, nv + 1):
        label = joint_names[j - 1] if joint_names else f"joint{j}"
        names.append(f"{label}_Fv")
        names.append(f"{label}_Fc")
    return names


def compute_base_parameters(Y_full, tol_ratio=1e-8):
    """
    SUTUN PIVOTLU QR AYRISTIRMASI ile Y_full matrisinin bagimsiz
    sutunlarini (base parameters) bulur.

    scipy.linalg.qr(..., pivoting=True), matrisin sutunlarini
    "en bilgilendiriciden en az bilgilendiriciye" siraya sokar ve
    R ust-ucgen matrisinin kosegen elemanlarinin buyuklugu, o sutunun
    ne kadar BAGIMSIZ bilgi tasidigini gosterir. Kosegen elemani
    (toleransa gore) sifira yakinsa, o sutun ONCEKI sutunlarin bir
    LINEER KOMBINASYONUDUR - yani ayri bir parametre olarak
    GOZLEMLENEMEZ.

    Donenler:
      independent_col_indices: Y_full'daki hangi sutunlarin bagimsiz
                                (identifiable) oldugu
      rank: toplam bagimsiz parametre sayisi
    """
    import scipy.linalg as sla

    Q_, R_, piv = sla.qr(Y_full, mode="economic", pivoting=True)
    diag_abs = np.abs(np.diag(R_))
    tol = tol_ratio * diag_abs[0]
    rank = int(np.sum(diag_abs > tol))

    independent_col_indices = sorted(piv[:rank].tolist())
    return independent_col_indices, rank


def fit_base_parameters(Y_full, tau_full, independent_col_indices):
    """Sadece BAGIMSIZ (identifiable) sutunlarla en kucuk kareler cozer.
    Digerleri icin bir sonuc URETILMEZ (cunku matematiksel olarak
    anlamsiz olurdu - "unidentifiable" olarak isaretlenir)."""
    Y_reduced = Y_full[:, independent_col_indices]
    phi_reduced, residuals, rank, sv = np.linalg.lstsq(Y_reduced, tau_full, rcond=None)
    return phi_reduced
