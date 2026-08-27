"""
05_apply_servo_gains_to_mjcf.py - Fit edilen kp/kv'yi MJCF'ye isler

KULLANIM: python scripts/05_apply_servo_gains_to_mjcf.py

Aktuator eslestirmesi isim desenine degil, MuJoCo'nun kendi baglanti
yapisina (trnid) bakarak yapilir - herhangi bir aktuator isimlendirme
semasiyla calisir.
"""

import json
import os
import sys
import xml.etree.ElementTree as ET

import mujoco

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src import config
from src import common


def main():
    with open(config.FITTED_SERVO_GAINS_JSON) as f:
        gains = json.load(f)

    # Aktuator isimlerini MuJoCo baglanti yapisindan bulmak icin modeli
    # gecici olarak yukluyoruz (mesh gerektirir - ama sadece isim/id
    # bilgisi icin, fizik simulasyonu yapmiyoruz, bu yuzden hizli).
    mj_model = mujoco.MjModel.from_xml_path(config.MJCF_WITH_FRICTION)

    tree = ET.parse(config.MJCF_WITH_FRICTION)
    root = tree.getroot()

    updated = []
    for joint_name, kv_kp in gains.items():
        act_id = common.find_position_actuator_for_joint(mj_model, joint_name)
        if act_id is None:
            print(f"[UYARI] '{joint_name}' icin aktuator bulunamadi, atlaniyor.")
            continue
        act_name = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_id)

        # XML agacinda ayni isimli <position> etiketini bul ve guncelle
        for act_elem in root.findall(".//position"):
            if act_elem.get("name") == act_name:
                act_elem.set("kp", f"{kv_kp['kp']:.2f}")
                act_elem.set("kv", f"{kv_kp['kv']:.2f}")
                updated.append(act_name)
                break

    if len(updated) != len(gains):
        print(f"[UYARI] Beklenen {len(gains)} aktuator yerine {len(updated)} guncellendi.")
    ET.indent(tree, space="  ")
    tree.write(config.MJCF_WITH_FRICTION_AND_GAINS, encoding="utf-8", xml_declaration=True)
    print(f"[bilgi] Guncellenen aktuatorler: {updated}")
    print(f"[bilgi] NIHAI model dosyasi: {config.MJCF_WITH_FRICTION_AND_GAINS}")


if __name__ == "__main__":
    main()
