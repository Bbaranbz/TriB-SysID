"""
run_pipeline.py - Tum Pipeline'i Tek Seferde Calistir (import-tabanli)
===========================================================================

subprocess yerine, her script'in main() fonksiyonunu DOGRUDAN import edip
cagirir - ayri Python surecleri baslatmadan, tek surecte calisir.

NOT: scripts/ klasorundeki dosya adlari rakamla basladigi icin
(01_validate_friction.py gibi) normal 'import' ifadesi kullanilamaz -
Python modul isimleri rakamla baslayamaz. Bu yuzden importlib.util ile
dosya yolundan dinamik yukleme yapiyoruz.
"""

import importlib.util
import os
import sys

SCRIPTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scripts")

PIPELINE = [
    "01_validate_friction.py",
    "02_apply_friction_to_urdf.py",
]


def load_module_from_path(script_path):
    """Rakamla baslayan dosya adlarini da (01_validate_friction.py gibi)
    normal import ifadesi kullanmadan, dosya yolundan yukler."""
    module_name = os.path.splitext(os.path.basename(script_path))[0]
    spec = importlib.util.spec_from_file_location(module_name, script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    for script_name in PIPELINE:
        script_path = os.path.join(SCRIPTS_DIR, script_name)
        print(f"\n{'='*70}\nCALISTIRILIYOR: {script_name}\n{'='*70}")
        try:
            module = load_module_from_path(script_path)
            module.main()
        except Exception as e:
            print(f"\n[HATA] {script_name} basarisiz oldu: {e}. Pipeline durduruldu.")
            sys.exit(1)

    print(f"\n{'='*70}\nPIPELINE TAMAMLANDI (URDF/fiziksel parametre tanimlama)\n{'='*70}")
    print("Ciktilar: outputs/fitted_friction.json, outputs/robot_with_friction.urdf, "
          "outputs/model_with_friction.xml")
    print("\nOPSIYONEL: MuJoCo-ozel servo kazanci (kp/kv) tanimlama icin (sadece MuJoCo")
    print("position-aktuator kullananlar icin gereklidir - cogu kullanici icin GEREKMEZ):")
    print("  optional_mujoco_servo_module/README.md dosyasina bakin.")


if __name__ == "__main__":
    main()