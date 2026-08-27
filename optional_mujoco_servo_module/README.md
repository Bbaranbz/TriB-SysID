# [OPSİYONEL] MuJoCo Servo Kazancı (kp/kv) Modülü

**Bu klasördeki hiçbir script, ana pipeline'ın (kütle/atalet/sürtünme
tanımlama) çalışması için gerekli değildir.** Ana `README.md`'de
anlatılan URDF-tabanlı sistem tanımlama tamamen bağımsız, herhangi bir
rijit-gövde sistemi için çalışır ve bu klasöre hiç ihtiyaç duymaz.

## Bu modül ne işe yarar, kimin işine yarar

Bu modül, **spesifik olarak MuJoCo'nun `<position>` aktüatör tipini**
kullanan bir simülasyon kuran kişiler için: aktüatörün `kp` (oransal) ve
`kv` (türevsel/sönümleme) kazançlarını, gerçek robotun target/actual
davranışına en çok benzeyecek şekilde bulur.

**Bu size uygun değilse (ve çoğu kullanıcı için uygun değildir) atlayın:**
- Başka bir simülatör kullanıyorsanız (Gazebo, PyBullet, Isaac Sim, vs.)
- MuJoCo kullanıyorsanız ama torque control veya farklı bir kontrol
  stratejisi uyguluyorsanız
- Sadece robotun **fiziksel** (kütle/atalet/sürtünme) parametrelerini
  URDF olarak elde etmek istiyorsanız — ana pipeline bunun için yeterli

## Neden ayrı bir modül

`kp`/`kv`, robotun fiziksel bir özelliği değil, **MuJoCo'nun kontrolcü
modelinin** bir parametresidir. Ana pipeline'daki kütle/atalet/sürtünme
tanımlama evrensel (herhangi bir rijit-gövde sistemine, herhangi bir
simülatörle uygulanabilir) kalsın diye, bu MuJoCo'ya özel kısım ayrı
tutulmuştur.

## Kullanım (eğer ihtiyacınız varsa)

Önce ana pipeline'ı (`python run_pipeline.py`, proje kökünden) çalıştırıp
`outputs/model_with_friction.xml` dosyasının üretildiğinden emin olun.
Sonra:

```bash
python optional_mujoco_servo_module/01_identify_servo_gains.py
python optional_mujoco_servo_module/02_apply_servo_gains_to_mjcf.py
```

Doğrulama için:
```bash
python optional_mujoco_servo_module/03_simulate_and_compare.py --model full
```

Detaylı yöntem açıklaması için script'lerin içindeki docstring'lere
bakın (ana `README.md`'deki "Metodoloji Özeti" bölümünde de kısaca
anlatılmıştır).
