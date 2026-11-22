# Robotsuz Simülasyon (FTC 24230)

Bilgisayarda, robot olmadan:
- şasiyi kumandayla sürebilir,
- otonom path'lerini çalıştırabilir,
- hepsini **AdvantageScope**'ta 2D/3D sahada canlı izleyebilir ve kaydedebilirsin.

Simülatör **gerçek Pedro Follower'ını** robottaki **Foresight katsayılarıyla** çalıştırır. Motorlar ve Pinpoint yerine bir fizik modeli kullanılır. Robottaki sürüş kodu (`TeleopDrive`) ve otonom path'leri (`AutoPaths`) simülatörle **aynı dosyalardan** gelir. Simde ne değiştirirsen robotta da aynı şekilde çalışır.

---

## Yapı

```
shared/   Robot + sim ORTAK kodu (saf Java, Android sınıfı YOK)
  PedroTuning.java   Foresight katsayıları (tek kaynak)
  TeleopDrive.java   Sürüş mantığı
  AutoPaths.java     Otonom path'leri
  DriverInput.java   Gamepad'in donanımdan bağımsız hâli
sim/      Sadece bilgisayarda çalışır, robota YÜKLENMEZ
  SimMain.java       Ana döngü
  SimConfig.java     Fizik/ayar sabitleri
  MecanumPhysics, SimDrivetrain, SimLocalizer, DesktopGamepad, log/
TeamCode/.../sim/
  RobotDriveTeleop   Robotta: "Drive (Sim ile ayni)"
  RobotExampleAuto   Robotta: "Ornek Otonom (Sim ile ayni)"
```

**Kural:** `shared/` içinde `HardwareMap`, `DcMotor`, `Gamepad`, `ElapsedTime` gibi FTC SDK sınıfları kullanılamaz. Kullanırsan simülatör derlenmez.

---

## İlk kurulum (bir kez)

1. **AdvantageScope**'u kur (26.x veya daha yeni).
2. Android Studio'da projeyi aç ve **Gradle Sync** yap.
3. Kumandayı (Xbox / PS4 / PS5) USB ya da Bluetooth ile bilgisayara bağla.

## Çalıştırma

Android Studio'nun alt kısmındaki **Terminal** sekmesinde:

```
gradlew.bat :sim:run                       (Windows)
./gradlew :sim:run                          (Mac / Linux)
```

Kumanda olmadan, sadece otonomu tekrar tekrar izlemek için:

```
gradlew.bat :sim:run --args="--auto"
```

Kapatmak için terminalde **Ctrl+C**.

## AdvantageScope'a bağlanma

1. Simülatör çalışırken AdvantageScope'ta **File > Connect to Simulator > RLOG Server**.
2. **+** ile bir **3D Field** (ya da 2D Field) sekmesi aç ve sağ alttan güncel FTC sahasını seç.
3. Soldaki listeden sürükle:
   - `Drive/Pose` → **Robot**
   - `Auto/FullRoute` → **Trajectory** (otonomun tüm rotası)
   - `Follower/CurrentPath` → **Trajectory** (o an takip edilen path)
4. Grafik için bir **Line Graph** sekmesi aç. `Drive/FieldVelocity` ve `Drive/WheelPowers` gibi alanları sürükle.

**Robot sahada döndürülmüş görünüyorsa:** Pedro köşe orijinli, AdvantageScope'un FTC sahası ise merkez orijinli çalışıyor. `SimConfig.PEDRO_TO_FIELD_ROTATION_DEG` değerini 0, 90, 180 ve 270 arasında dene. Robotu ileri sürdüğünde sahada da ileri gidiyorsa doğru değeri bulmuşsun demektir.

## Kumanda

| Tuş | Görev |
|---|---|
| Sol çubuk | İleri/geri + yanlara |
| Sağ çubuk (X) | Dönüş |
| LB (basılı tut) | Yavaş mod |
| Back | Field-centric aç/kapa |
| Start | Field-centric için "ileri"yi şu anki yön yap |
| **A** | Başlangıca koy ve otonomu başlat (sadece sim) |
| **B** | Otonomu iptal et (çubuğu oynatmak da iptal eder) |
| **Y** | Başlangıç noktasına ışınla (sadece sim) |

Çubuk yukarıyken robot geri gidiyorsa `SimConfig.INVERT_STICK_Y` değerini değiştir.

## Kayıtlar

Her çalıştırma `sim/logs/sim_TARIH.rlog` dosyasına kaydedilir. Sonradan incelemek için AdvantageScope'ta **File > Open Log** ile açabilirsin.

## Simülasyonu gerçek robota yaklaştırmak

`SimConfig.java` içinde:
- `MAX_ANGULAR_VELOCITY`: robot tam güçte ne kadar hızlı dönüyor (rad/s)
- `DRIVE_TIME_CONSTANT`: hızlanma ne kadar ağır (büyük değer = daha ağır)
- `POSE_NOISE_INCHES`: Pinpoint gürültüsü (0 = kusursuz)

Maksimum hızlar ve doğal yavaşlama otomatik olarak `PedroTuning`'deki tuner sonuçlarından alınıyor.

## Robotta deneme

Aynı kod robotta iki OpMode olarak var:
- **Drive (Sim ile ayni)**: TeleOp
- **Ornek Otonom (Sim ile ayni)**: Otonom. **Önce robotu `AutoPaths.START` konumuna ve yönüne koy.**
