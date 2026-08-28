# TriB-SysID — Genel Rijit-Gövde Sistem Tanımlama Aracı

![Python](https://img.shields.io/badge/python-3.10%2B-blue)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)
![Robotics](https://img.shields.io/badge/domain-rigid--body%20dynamics-informational)

**Herhangi bir açık-zincir, rijit-gövde eklemli sisteme** (robot kolu, çift
sarkaç, herhangi bir seri manipülatör) uygulanabilecek, gerçek sensör
verisinden **kütle/atalet parametrelerini ve sürtünmeyi** tanımlayıp bunları
güncellenmiş bir **URDF** olarak üreten, uçtan uca doğrulanmış bir sistem
tanımlama (system identification) aracı.

Bu depo başlangıçta belirli bir 6 eksenli robot kolu için geliştirilmiş, sonra
**hiçbir robota özgü varsayım kalmayacak şekilde genelleştirilmiştir**: eklem
sayısı, eklem isimleri — hepsi URDF dosyanızdan **otomatik okunur**. Kod
içinde hiçbir yerde sabit bir eklem sayısı veya isim şeması yoktur.

**Bu araç hiçbir simülatöre veya kontrolcü mimarisine bağımlı değildir** —
çıktısı standart bir URDF dosyasıdır, istediğiniz simülatörle
(Gazebo, PyBullet, Isaac Sim, MuJoCo, ya da doğrudan gerçek robot) kullanabilirsiniz.
(MuJoCo kullananlar için ayrıca opsiyonel bir servo-kazancı modülü de
mevcuttur — bkz. aşağıdaki "Opsiyonel Modüller" bölümü.)

## Hızlı Başlangıç (kendi robotunuz/sisteminiz için)

1. **Kendi URDF dosyanızı** `models/robot.urdf` olarak kaydedin (kaç eklemi
   olursa olsun, eklemleri ne isim taşırsa taşısın çalışır).
2. Robottan/sisteminizden topladığınız trajectory kayıtlarını (aşağıdaki
   "Veri Formatı" bölümüne uygun CSV dosyaları) `data/raw/` klasörüne atın.
3. Kurulum ve çalıştırma adımlarını takip edin.

Script'ler `data/raw/` klasörünü her çalıştırıldığında **yeniden tarar** —
gerekli sütunlara sahip olmayan dosyaları otomatik atlar, hata vermez.

## Kurulumu Doğrulama (GitHub'a yüklemeden/kod değiştirmeden önce)

`examples/synthetic_3dof_arm/` klasöründe, **bilinen (ground truth)**
parametrelere sahip sentetik bir test sistemi bulunur. Pipeline'ın doğru
çalıştığından emin olmak için:

```bash
cp examples/synthetic_3dof_arm/robot.urdf models/robot.urdf
cp examples/synthetic_3dof_arm/trajectory_log.csv data/raw/trajectory_log.csv
python run_pipeline.py
python examples/synthetic_3dof_arm/compare_with_ground_truth.py
```

Detaylar için `examples/synthetic_3dof_arm/README.md`'ye bakın.

## Kurulum

```bash
pip install -r requirements.txt
```

**Geliştirici modunda kurulum (önerilir):** Proje standart bir Python paketi
olarak (`pyproject.toml` ile) da kurulabilir. Bu, `src/` klasöründeki
`config` ve `common` modüllerini, script'lerin bulunduğu klasörden bağımsız
olarak doğrudan `from src import config` şeklinde import edebilmenizi sağlar
(hiçbir `sys.path` düzenlemesi gerekmeden):

```bash
pip install -e .
```

(`-e` = editable/geliştirici modu; kaynak kodda değişiklik yaptığınızda
paketi yeniden kurmanıza gerek kalmaz, değişiklikler anında yansır.)

## Klasör Yapısı

```
lebai_sysid_project/
├── src/
│   ├── __init__.py
│   ├── common.py               <- Paylaşılan fonksiyonlar (filtreleme, RNEA, fit)
│   └── config.py                <- TÜM dosya yolları burada tanımlı
├── scripts/                          <- ANA (evrensel, simülatör-bağımsız) pipeline
│   ├── 00_full_parameter_identification.py  <- opsiyonel: tam kütle/atalet tanımlama
│   ├── 01_validate_friction.py
│   ├── 02_apply_friction_to_urdf.py
│   └── 03_apply_friction_to_mjcf.py          <- MJCF'ye de işlemek isteyenler için
├── optional_mujoco_servo_module/      <- [OPSİYONEL] Sadece MuJoCo kullananlar için
│   ├── README.md
│   ├── 01_identify_servo_gains.py
│   ├── 02_apply_servo_gains_to_mjcf.py
│   └── 03_simulate_and_compare.py
├── tests/                              <- Birim testleri (pytest)
│   ├── __init__.py
│   └── test_common.py
├── notebooks/                          <- Jupyter defterleri / veri ön işleme
├── examples/
│   └── synthetic_3dof_arm/            <- Doğruluk kanıtlama örneği
├── models/
│   └── robot.urdf            <- Robotunuzun/sisteminizin URDF'si (buraya koyun)
├── data/
│   └── raw/                   <- YENİ CSV DOSYALARINI BURAYA ATIN
├── outputs/                   <- Tüm çıktılar otomatik burada oluşur
│   ├── fitted_friction.json
│   ├── robot_with_friction.urdf      <- ASIL HEDEF CIKTI
│   └── base_parameters_report.json   <- (opsiyonel tam parametre modu icin)
├── pyproject.toml
├── requirements.txt
├── run_pipeline.py          <- Ana (URDF/fiziksel parametre) pipeline'ı çalıştırır
├── LICENSE
└── README.md
```

## Kullanım

### Tüm ana pipeline'ı tek seferde çalıştırmak

```bash
python run_pipeline.py
```

### Adım adım çalıştırmak (önerilir — her adımın çıktısını görmek için)

```bash
python scripts/01_validate_friction.py
python scripts/02_apply_friction_to_urdf.py
python scripts/03_apply_friction_to_mjcf.py   # opsiyonel, MJCF de isterseniz
```

Sonuç: `outputs/robot_with_friction.urdf` — kütle/atalet (URDF'deki mevcut
değerler) + tanımlanan sürtünme (`<dynamics damping="..." friction="..."/>`)
içeren, kullanıma hazır bir URDF dosyası.

### Testleri çalıştırmak

```bash
pytest tests/
```

## Kullanım Senaryoları

Bu araç, rijit-gövde dinamiğine dayanan gerçek mühendislik problemlerinde doğrudan uygulanabilir:

### Endüstriyel CNC ve İşleme (Machining) Sistemleri

Mekanik atölye şartlarında üretilen veya modifiye edilen CNC eksenlerinde (doğrusal kayar eksenler ve dönel eksenlerin karışımı), üretime geçilmeden önce sürtünme kompanzasyonu yapılması ve servo motor kazançlarının (kp/kv) gerçek sensör verisiyle doğrulanması. Bu araç, hem doğrusal (prismatic) hem dönel (revolute) eksenleri karışık olarak destekler — bir CNC torna/freze makinesinin tipik eksen konfigürasyonu tam bu kapsama girer. Sürtünme kompanzasyonunun doğru kalibre edilmesi, işleme hassasiyetini doğrudan etkiler; özellikle düşük hızlı, hassas kontur işlemlerinde Coulomb sürtünmesinin doğru tanımlanması kritik önem taşır.

### Otonom Manipülatörler ve Robot Kolları

Üretim hattında çalışan çok eksenli robot kollarında, zamanla aşınmaya bağlı olarak değişen eklem sürtünmelerinin (viskoz ve Coulomb) periyodik olarak yeniden kestirilmesi ve modelin güncel tutulması. Tanımlanan güncel parametrelerle MuJoCo gibi bir simülatörde dijital ikiz (digital twin) oluşturularak, yeni bir hareket planının veya kontrol algoritmasının gerçek robota yüklenmeden önce simülasyonda test edilmesi — bu, hem geliştirme süresini kısaltır hem de gerçek donanımda deneme-yanılma riskini azaltır.

## Gelecek Özellikler (Planlanan)

### CAD-Düzenlileştirilmiş (Regularized) Tam Parametre Tanımlama

Şu anki `00_full_parameter_identification.py`, zayıf gözlemlenebilen atalet
parametrelerini (bkz. "Bilinen Sınırlamalar") güvenilir şekilde ayıramıyor.
Planlanan iyileştirme: tüm parametreleri **aynı anda**, iki hedefi dengeleyen
bir optimizasyonla çözmek —

1. Veriye mümkün olduğunca iyi uysun (mevcut yöntem gibi)
2. **Aynı zamanda** URDF'deki mevcut (üretici/CAD) değerlerinden çok fazla
   sapmasın (bir düzenlileştirme/ceza terimi ile)

Bu sayede iyi gözlemlenebilen parametreler (sürtünme gibi) veriye göre serbestçe
ayarlanırken, zayıf gözlemlenebilen parametreler otomatik olarak CAD değerine
yakın kalır — hem fiziksel olarak tutarlı hem kullanılabilir bir sonuç elde
edilir. Bu yöntem literatürde bilinen bir teknik olup (bkz. Traversaro ve ark.,
fiziksel tutarlılık kısıtlı parametre tanımlama), muhtemelen `cvxpy` gibi bir
konveks optimizasyon kütüphanesi gerektirecektir.

## Opsiyonel Modüller

### `optional_mujoco_servo_module/` — Sadece MuJoCo `<position>` aktüatörü kullananlar için

`kp`/`kv` servo kazançları robotun **fiziksel** bir özelliği değil, MuJoCo'nun
belirli bir aktüatör modelinin parametresidir — bu yüzden ana pipeline'dan
ayrı, tamamen opsiyonel bir modül olarak tutulmuştur. Başka bir simülatör
kullanıyorsanız veya sadece fiziksel parametrelerle (URDF) ilgileniyorsanız
bu klasöre hiç ihtiyacınız yok. Detaylar için `optional_mujoco_servo_module/README.md`.

### `scripts/00_full_parameter_identification.py` — Tam kütle/atalet tanımlama

Kütle/atalet parametrelerinin de (URDF'deki mevcut değerlere güvenmek yerine)
veriden tahmin edilmesini sağlayan genel bir regresör tabanlı tanımlama:

```bash
python scripts/00_full_parameter_identification.py
```

**Yöntem:** `pin.computeJointTorqueRegressor()` ile her link için 10 atalet
parametresi (kütle, kütle merkezi×3, atalet tensörü×6) ve her eklem için
sürtünme (Fv, Fc) parametrelerini içeren birleşik bir regresör matrisi
kurulur. Sütun pivotlu QR ayrıştırması ile bu ham parametrelerin hangi
**bağımsız kombinasyonlarının** (base parameters) veriden gerçekten
gözlemlenebilir olduğu belirlenir — geri kalanlar için sonuç uydurulmaz.

**⚠️ Kritik uyarı:** Bu modun güvenilir sonuç vermesi için, sürtünme
tanımlamada kullanılan basit trajectory'ler **yetersizdir**. Script,
sonuçların fiziksel olarak makul olup olmadığını (negatif kütle/atalet,
aşırı büyük değerler) otomatik kontrol eder ve **uyarır** — eğer uyarı
alırsanız, bu sonuçları modele işlemeyin, kütle/atalet için URDF'deki
üretici değerlerini korumaya devam edin. Güvenilir tam parametre
tanımlama için, robotun çok geniş bir konfigürasyon/hız/ivme uzayını
tarayan, özel olarak optimize edilmiş ("persistently exciting")
trajectory'lerle yeniden veri toplamanız gerekir.

Çıktı: `outputs/base_parameters_report.json` — tanımlanabilirlik oranı,
bulunan parametre değerleri, ve fiziksel tutarlılık uyarılarını içerir.

## Veri Kalitesi Gereksinimleri

Sürtünme tanımlama için `actual_q`, `actual_qd`, `actual_tau` sütunları
yeterlidir. Hız çeşitliliği ne kadar genişse (yavaş VE hızlı hareketler bir
arada) `Fv`/`Fc` ayrışması o kadar güvenilir olur.

## Veri Formatı (CSV Kuralı)

Eklem isimlerinden **bağımsız, evrensel** bir sayısal indeksleme kullanılır.
Script'ler URDF'nizdeki eklemleri **tanımlandıkları sırayla** okur (örneğin
URDF'de `shoulder`, sonra `elbow` tanımlıysa, `shoulder` → index 1, `elbow`
→ index 2 demektir):

| Sütun | Açıklama |
|---|---|
| `time_s` | Zaman damgası (saniye) — opsiyonel, yoksa 100 Hz varsayılır |
| `actual_q1, actual_q2, ...` | Gerçek eklem pozisyonu (rad), URDF sırasına göre |
| `actual_qd1, actual_qd2, ...` | Gerçek eklem hızı (rad/s) |
| `actual_tau1, actual_tau2, ...` | Gerçek ölçülen eklem torku (Nm) |
| `target_q1, target_q2, ...` | Hedeflenen eklem pozisyonu (sadece `optional_mujoco_servo_module` için gerekli) |

**Not:** Bu, kod içinde bulunan tek "isimlendirme kuralı" varsayımıdır —
eklemlerin kendi isimleri (URDF'deki `name="..."` özelliği) tamamen
serbesttir, sadece CSV sütunlarının SIRA numarasıyla eşleşmesi gerekir.

## Metodoloji Özeti

1. **Sürtünme:** URDF'deki bilinen kütle/atalet parametreleriyle RNEA
   (Pinocchio) kullanılarak sürtünmesiz beklenen tork hesaplanır. Gerçek
   ölçülen tork ile arasındaki fark (residual), `Fv·qd + Fc·sign(qd)`
   modeline sınırlı en küçük kareler ile fit edilir.
2. **Doğrulama:** (a) Residual'in hız-yönüyle güçlü, pozisyonla zayıf
   korele olduğu kontrol edilir (gerçekten sürtünme mi, model hatası mı).
   (b) Dosya-farkındalı train/test split ile ezberleme kontrolü yapılır.

## Bilinen Sınırlamalar

- Sürtünme modeli basit (viskoz+Coulomb) — Stribeck etkisi (düşük hızda
  sürtünmenin azalması) modellenmemiştir; çok dar hız aralıklı verilerde
  bu, `Fv` tahmininde sapmaya yol açabilir.
- Tam parametre tanımlama modu (`00_...`), yetersiz uyarma trajectory'leriyle
  fiziksel olarak anlamsız sonuçlar üretebilir — script bunu otomatik
  tespit edip uyarır, ama nihai kararı kullanıcı vermelidir.
- `scripts/00_full_parameter_identification.py`'nin raporladığı **atalet/kütle-merkezi
  parametre isimleri yanıltıcı olabilir**: QR ayrıştırması, birbirine bağımlı
  (korele) ham parametreleri elediğinde, kalan sütuna verdiği katsayı o
  parametrenin saf kendisi değil, elenen parametrelerin etkisini de içeren bir
  **doğrusal kombinasyon** olabilir. Bu script'ten **sadece sürtünme (Fv, Fc)
  çıktılarına güvenin** — kütle/atalet için raporlanan sayısal değerleri ham
  gerçek fiziksel değer olarak yorumlamayın. (Bu durum, bilinen ground-truth
  parametreli sentetik bir testle doğrulanmıştır: sürtünme değerleri %1-6 hata
  ile doğru çıkarken, bir atalet-kombinasyon değeri gerçek değerden ~7 kat
  sapmıştır.)

## Lisans

Bu proje [MIT Lisansı](LICENSE) altında yayınlanmıştır.
