"""
03_apply_friction_to_mjcf.py - Fit edilen surtunmeyi MJCF'ye isler

KULLANIM: python scripts/03_apply_friction_to_mjcf.py
"""

import json
import os
import sys
import xml.etree.ElementTree as ET

from src import config
from src import common

def main():
    with open(config.FITTED_FRICTION_JSON) as f:
        friction_params = json.load(f)

    tree = ET.parse(config.MJCF_PATH)
    root = tree.getroot()

    # ONEMLI: meshdir'i MUTLAK yola cevir. Orijinal MJCF, mesh'leri
    # KENDI klasorune GORE (goreceli, "meshdir=meshes") buluyordu. Bizim
    # isledigimiz kopya outputs/ klasorune yazilacagi icin, o klasorde
    # "meshes/" alt klasoru YOK - dosya calismaz. Mutlak yola cevirerek,
    # bu dosya nereye kopyalanirsa kopyalansin calismasini saglariz.
    compiler_elem = root.find("compiler")
    if compiler_elem is not None:
        abs_meshdir = os.path.join(config.MODELS_DIR, "meshes")
        compiler_elem.set("meshdir", abs_meshdir)

    updated = []
    for joint_elem in root.findall(".//joint"):
        name = joint_elem.get("name")
        if name in friction_params:
            p = friction_params[name]
            joint_elem.set("damping", f"{p['Fv']:.6f}")
            joint_elem.set("frictionloss", f"{p['Fc']:.6f}")
            updated.append(name)

    if len(updated) != len(friction_params):
        print(f"[UYARI] fitted_friction.json'da {len(friction_params)} eklem var, "
              f"ama MJCF'de bunlarin sadece {len(updated)} tanesi bulundu/guncellendi. "
              f"Eklem isimleri (JSON vs MJCF) uyusmuyor olabilir.")

    ET.indent(tree, space="  ")
    tree.write(config.MJCF_WITH_FRICTION, encoding="utf-8", xml_declaration=True)    
    print(f"[bilgi] Guncellenen eklemler: {updated}")
    print(f"[bilgi] Yeni dosya: {config.MJCF_WITH_FRICTION}")


if __name__ == "__main__":
    main()
