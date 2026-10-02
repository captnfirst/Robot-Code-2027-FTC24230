package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Superstructure {
    public static Vision vision = new Vision();

    private static final PIDController visionPID = new PIDController(0.02, 0.0, 0.0);

    // Tolerans: ±1.2 derece (Limelight tx gürültüsünü filtrelemek için)
    private static final double ALIGNMENT_TOLERANCE_DEG = 2;

    // Minimum motor gücü: Şasenin sürtünmesini yenip küçük açılarda takılmasını önler (feedforward/kF )
    private static final double MIN_TURN_POWER = 0.00;

    // Radar / Sweep Ayarları
    private static final double SWEEP_AMPLITUDE = Math.toRadians(30.0); // ±30 Derece
    private static final double SWEEP_POWER = 0.15;

    private static boolean isSweeping = false;
    private static double centerHeading = 0.0;
    private static int sweepDirection = 1;

    public static void init(HardwareMap hardwareMap) {
        vision.init(hardwareMap);

        visionPID.setSetPoint(0.0);
        visionPID.setTolerance(ALIGNMENT_TOLERANCE_DEG);
    }

    public static void periodic() {
        vision.periodic();
    }

    public static void read() {
        vision.read();
    }

    public static void write() {
        vision.write();
    }

    public static void reset() {
        vision.reset();
        visionPID.reset();
        isSweeping = false;
    }

    public static void stopSweep() {
        isSweeping = false;
    }

    /**
     * Tag varsa PID ile kilitlenir, yoksa ±30 derece radar taraması yapar.
     * @param currentHeading Robotun anlık açı değeri (Radyan cinsinden)
     * @return Motor dönüş gücü
     */
    public static double updateTargetLockOrSweep(double currentHeading) {
        // DURUM 1: Tag Görünüyor -> Kilitlenme Mantığı
        if (vision.tv) {
            isSweeping = false;

            // 1. Gerçek tx açısına göre tolerans kontrolü:
            // Hedef zaten merkezdeyse motoru durdur
            if (Math.abs(vision.tx) <= ALIGNMENT_TOLERANCE_DEG) {
                return 0.0;
            }

            // 2. PID çıktısını hesapla (SetPoint zaten 0.0)
            // tx pozitifse (hedef sağdaysa) robot sağa dönecek şekilde güç üretir
            double turnPower = visionPID.calculate(vision.tx, 0.0);

            // 3. Minimum sürtünme desteği (küçük açılarda robotun donup kalmasını önler)
            if (turnPower > 0) {
                turnPower += MIN_TURN_POWER;
            } else if (turnPower < 0) {
                turnPower -= MIN_TURN_POWER;
            }

            // 4. Güvenli maksimum sınır [-0.55, 0.55]
            return Math.max(-0.35, Math.min(0.35, turnPower));
        }

        // DURUM 2: Tag Görünmüyor -> ±30° Radar Taraması
        if (!isSweeping) {
            centerHeading = currentHeading;
            isSweeping = true;
            sweepDirection = 1;
        }

        double headingError = normalizeAngle(currentHeading - centerHeading);

        if (headingError >= SWEEP_AMPLITUDE) {
            sweepDirection = -1;
        } else if (headingError <= -SWEEP_AMPLITUDE) {
            sweepDirection = 1;
        }

        return SWEEP_POWER * sweepDirection;
    }

    /**
     * Gerçek tx verisine dayalı kesin hizalanma kontrolü
     */
    public static boolean isAligned() {
        return vision.tv && Math.abs(vision.tx) <= ALIGNMENT_TOLERANCE_DEG;
    }

    private static double normalizeAngle(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}