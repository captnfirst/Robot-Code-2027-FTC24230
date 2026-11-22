package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.shared.PedroTuning;

/** Simülasyonun fiziksel ayarları. Gerçek robota benzemesi için buradaki sayılarla oynayabilirsin. */
public final class SimConfig {
    private SimConfig() {}

    // ---- Döngü ----
    /** Ana döngü periyodu (s). Gerçek robottaki döngü süresine yakın tut. */
    public static double LOOP_PERIOD = 0.020;
    /** Fizik alt adımı (s). Küçük = daha doğru. */
    public static double PHYSICS_STEP = 0.002;

    // ---- Şasi (inç, inç/s, inç/s², rad/s) ----
    /** Robot ayak izi: yarı genişlik/uzunluk (duvar çarpışması için). 18" robot -> 9. */
    public static double ROBOT_HALF_SIZE = 9.0;
    public static double FIELD_SIZE = 144.0;

    /** Tam güçte ulaşılan hızlar: Pedro tuner'ının ölçtüğü değerler. */
    public static double MAX_FORWARD_VELOCITY = PedroTuning.MAX_FORWARD_VELOCITY;
    public static double MAX_STRAFE_VELOCITY = PedroTuning.MAX_STRAFE_VELOCITY;
    /** Tam güçte dönüş hızı. Yaklaşık: ileri hız / (iz genişliği + dingil mesafesi)/2 */
    public static double MAX_ANGULAR_VELOCITY = 5.5;

    /** Güç verildiğinde hızın hedefe yaklaşma zaman sabiti (s). Büyük = daha ağır hızlanma. */
    public static double DRIVE_TIME_CONSTANT = 0.25;
    /** BRAKE modunda güç 0 iken durma zaman sabiti (s). */
    public static double BRAKE_TIME_CONSTANT = 0.12;
    /** FLOAT modunda güç 0 iken sürtünme yavaşlaması: tuner'ın ölçtüğü doğal yavaşlama. */
    public static double COAST_FORWARD_DECEL = PedroTuning.NATURAL_FORWARD_DECEL;
    public static double COAST_STRAFE_DECEL = PedroTuning.NATURAL_STRAFE_DECEL;
    public static double COAST_ANGULAR_DECEL = 6.0;

    /** Teleop'ta güç 0 olunca frenle (Constants.drivetrainConfig.manualBrakeMode ile aynı). */
    public static boolean MANUAL_BRAKE_MODE = true;

    // ---- Lokalizasyon gürültüsü (gerçekçilik için, 0 = kusursuz Pinpoint) ----
    public static double POSE_NOISE_INCHES = 0.0;
    public static double HEADING_NOISE_RAD = 0.0;

    // ---- AdvantageScope ----
    /** AdvantageScope'taki FTC sahası merkez orijinli, metre. Pedro köşe orijinli, inç.
     *  Robot AdvantageScope'ta sahada döndürülmüş görünüyorsa bunu 0 / 90 / 180 / 270 yap. */
    public static double PEDRO_TO_FIELD_ROTATION_DEG = 0.0;
    public static int RLOG_PORT = 5800;
    public static boolean WRITE_LOG_FILE = true;

    // ---- Gamepad ----
    /** Çubuk yukarıyken robot geri gidiyorsa bunu değiştir. */
    public static boolean INVERT_STICK_Y = false;
}
