"""
02_apply_friction_to_urdf.py - Fit edilen surtunmeyi URDF'ye isler

KULLANIM: python scripts/02_apply_friction_to_urdf.py
(Onceden 01_validate_friction.py calistirilmis olmali)
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

    tree = ET.parse(config.URDF_PATH)
    root = tree.getroot()

    updated = []
    for joint_elem in root.findall(".//joint"):
        name = joint_elem.get("name")
        if name in friction_params:
            p = friction_params[name]
            existing = joint_elem.find("dynamics")
            if existing is not None:
                joint_elem.remove(existing)
            dyn = ET.SubElement(joint_elem, "dynamics")
            dyn.set("damping", f"{p['Fv']:.6f}")
            dyn.set("friction", f"{p['Fc']:.6f}")
            updated.append(name)

    if len(updated) != len(friction_params):
        print(f"[UYARI] fitted_friction.json'da {len(friction_params)} eklem var, "
              f"ama URDF'de bunlarin sadece {len(updated)} tanesi bulundu/guncellendi. "
              f"Eklem isimleri (JSON vs URDF) uyusmuyor olabilir.")

    ET.indent(tree, space="  ")
    tree.write(config.URDF_WITH_FRICTION, encoding="utf-8", xml_declaration=True)
    print(f"[bilgi] Guncellenen eklemler: {updated}")
    print(f"[bilgi] Yeni dosya: {config.URDF_WITH_FRICTION}")


if __name__ == "__main__":
    main()
