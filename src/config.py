"""
config.py - Merkezi Konfigurasyon
=====================================

Bu dosya, projedeki TUM dosya yollarinin tek kaynagi (single source of
truth). Hicbir script icinde yol (path) sabit yazilmaz - hepsi buradan
import edilir. Boylece:

  - Proje klasoru tasindiginda hicbir script bozulmaz (hepsi ROOT_DIR'e
    gore otomatik hesaplanir).
  - Yeni bir veri dosyasi eklemek icin hicbir .py dosyasini ACMANIZA
    GEREK YOK - sadece dosyayi data/raw/ klasorune atmaniz yeterli,
    scriptler o klasordeki TUM gecerli CSV dosyalarini OTOMATIK bulur.

KULLANIM (yeni kullanicilar icin):
  1. Robotunuzun URDF dosyasini models/robot.urdf olarak kaydedin.
  2. MJCF dosyanizi models/ klasorune, mesh'lerinizi models/meshes/
     altina koyun (MJCF_PATH asagida ayarlayin).
  3. Robottan topladiginiz her CSV kaydini data/raw/ klasorune atin.
  4. python scripts/01_validate_friction.py calistirin - bu script
     data/raw/ icindeki TUM uygun dosyalari otomatik bulup kullanacak.
"""

import os

# --- Kok dizin: bu dosyanin bulundugu klasor ---
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# --- Klasor yollari ---
DATA_DIR = os.path.join(ROOT_DIR, "data", "raw")
MODELS_DIR = os.path.join(ROOT_DIR, "models")
OUTPUTS_DIR = os.path.join(ROOT_DIR, "outputs")

# --- Model dosyalari (bunlari kendi robotunuza gore degistirin) ---
URDF_PATH = os.path.join(MODELS_DIR, "robot.urdf")
MJCF_PATH = os.path.join(MODELS_DIR, "model.xml")  # kendi MJCF dosyanizin adiyla degistirin

# --- Cikti dosyalari (scriptler bunlari otomatik uretir) ---
FITTED_FRICTION_JSON = os.path.join(OUTPUTS_DIR, "fitted_friction.json")
FITTED_SERVO_GAINS_JSON = os.path.join(OUTPUTS_DIR, "fitted_servo_gains.json")
URDF_WITH_FRICTION = os.path.join(OUTPUTS_DIR, "robot_with_friction.urdf")
MJCF_WITH_FRICTION = os.path.join(OUTPUTS_DIR, "model_with_friction.xml")
MJCF_WITH_FRICTION_AND_GAINS = os.path.join(OUTPUTS_DIR, "model_with_friction_and_gains.xml")

# --- Veri kalitesi esikleri ---
# Bir CSV dosyasinin kp/kv tanimlamada kullanilabilmesi icin robotun
# kendi target/actual takip hatasinin bu esigin ALTINDA olmasi gerekir
# (rad cinsinden). Bu esigin uzerindeki dosyalar OTOMATIK DISLANIR.
OWN_TRACKING_ERROR_THRESHOLD_RAD = 0.05

# --- Filtreleme parametreleri ---
LOWPASS_CUTOFF_HZ = 10.0
LOWPASS_CUTOFF_HZ_ACCEL = 5.0  # ivme icin daha agresif filtre
FILTER_ORDER = 4  # Butterworth alcak-gecirgen filtre derecesi (varsayilan)

# Klasorlerin var oldugundan emin ol (yoksa olustur)
os.makedirs(DATA_DIR, exist_ok=True)
os.makedirs(OUTPUTS_DIR, exist_ok=True)

# --- TAM PARAMETRE TANIMLAMA (opsiyonel, genisletilmis mod) ---
# False birakilirsa sistem eskisi gibi SADECE surtunme+servo kazanci
# tanimlar (kutle/atalet "bilinen/guvenilir" kabul edilir - varsayilan).
# True yapilirsa, kutle/atalet/kutle-merkezi parametreleri de HAM
# REGRESOR ile (base parameters araciligiyla) tahmin edilmeye calisilir.
#
# DIKKAT: Bunu True yapmadan once README'deki "Tam Parametre Tanimlama"
# bolumunu MUTLAKA okuyun - cok daha zengin/genis bir uyarma trajectorisi
# gerektirir, aksi halde base parameter tahminleri anlamsiz cikar.
IDENTIFY_FULL_INERTIAL_PARAMS = False

FITTED_INERTIAL_PARAMS_JSON = os.path.join(OUTPUTS_DIR, "fitted_inertial_params.json")
BASE_PARAMS_REPORT_JSON = os.path.join(OUTPUTS_DIR, "base_parameters_report.json")
