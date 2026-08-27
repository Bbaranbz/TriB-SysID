# Sentetik Doğrulama Örneği: 3-DOF Düzlemsel Kol

Bu klasör, ana pipeline'ın **doğruluğunu kanıtlamak** için hazırlanmış,
bilinen (ground truth) parametrelere sahip sentetik bir test sistemidir.
GitHub'a yüklemeden önce (veya kodda değişiklik yaptıktan sonra) pipeline'ın
hâlâ doğru çalıştığını doğrulamak için kullanın.

## Nasıl çalışır

`generate_example_data.py`, `Fv=0.45, Fc=1.20, kp=300, kv=30` gibi
**önceden bilinen** sürtünme ve servo kazancı değerlerine sahip 3 eklemli
bir robot kurup, gerçek bir kapalı-döngü MuJoCo simülasyonu çalıştırır ve
çıkan hareketi gerçek bir robottan geliyormuş gibi `trajectory_log.csv`'ye
kaydeder. Bu bilinen değerler `ground_truth.json`'a da kaydedilir.

Ana pipeline bu veriyi işleyip `Fv`, `Fc`, `kp`, `kv` tahmin ettiğinde,
sonuçların `ground_truth.json`'daki değerlerle **örtüşmesi beklenir** —
örtüşmüyorsa, pipeline'da (veya yaptığınız bir değişiklikte) bir hata
var demektir.

## Kullanım

**1. Bu örneğin dosyalarını ana proje yapısına kopyalayın:**

```bash
cp examples/synthetic_3dof_arm/robot.urdf models/robot.urdf
cp examples/synthetic_3dof_arm/trajectory_log.csv data/raw/trajectory_log.csv
```

(`model_ground_truth.xml`, yalnızca `optional_mujoco_servo_module`'ü de test
etmek isterseniz gereklidir — o zaman ayrıca `models/model.xml` olarak
kopyalayın.)

**2. Pipeline'ı çalıştırın:**

```bash
python run_pipeline.py
```

**3. Sonuçları bilinen değerlerle karşılaştırın:**

```bash
python examples/synthetic_3dof_arm/compare_with_ground_truth.py
```

## Beklenen çıktı

```
=== SURTUNME KARSILASTIRMASI (Fv, Fc) ===
eklem       gercek Fv  bulunan Fv   hata%    gercek Fc  bulunan Fc   hata%
waist           0.450       0.443      1%        1.200       1.197      0%
shoulder        0.300       0.000    100%        0.850       0.739     13%
wrist           0.120       0.000    100%        0.350       0.299     15%

=== SERVO KAZANCI KARSILASTIRMASI (kp, kv) ===
eklem       gercek kp  bulunan kp   hata%    gercek kv  bulunan kv   hata%
waist           300.0       300.0      0%         30.0        30.0      0%
shoulder        220.0       220.0      0%         22.0        22.0      0%
wrist            90.0        90.0      0%          9.0         9.0      0%
```

**Servo kazançları için %0 hata** kesin bir doğrulamadır. **Sürtünmede**,
geniş hız aralığına sahip eklemlerde (`waist`) neredeyse tam örtüşme,
dar hız aralığına sahip eklemlerde ise `Fv`'nin sıfıra düşmesi (`Fc` yine
de makul kalır) beklenir — bu, **gerçek Lebai robot verisinde de
gözlemlenen** bir davranıştır (dar hız aralığında `Fv`/`sign(qd)` arasındaki
çoklu doğrusal bağlantı sorunu, bkz. ana `README.md`), pipeline'ın bir
hatası değildir.

## Neden MuJoCo'nun kendi `frictionloss`'unu kullanmadık

`generate_example_data.py`, sürtünmeyi MJCF'nin yerleşik `damping`/
`frictionloss` alanlarıyla değil, her simülasyon adımında **elle,
`qfrc_applied` üzerinden enjekte ederek** üretir. Bunun nedeni: MuJoCo'nun
`frictionloss`'u, ideal Coulomb sürtünmesi (`Fc·sign(qd)`) yerine kısıt
çözücüsü (constraint solver) tabanlı, dinamiğe bağlı farklı bir davranış
sergiliyor — bunu doğrudan test ederek doğruladık. Kendi ideal formülümüzü
enjekte ederek, ground truth'un tanımlama algoritmamızın varsaydığı
modelle **birebir tutarlı** olmasını sağladık; bu da temiz bir doğrulama
imkanı veriyor.
