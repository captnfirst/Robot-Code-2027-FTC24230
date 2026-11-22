package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.math.Pose;

/**
 * Basit mecanum şasi fizik modeli (Pedro koordinatları: inç, radyan, saha 0..144).
 *
 * Model: Tekerlek güçlerinden robotun hedef hızı hesaplanır (mecanum ileri kinematiği),
 * gerçek hız bu hedefe birinci dereceden bir gecikmeyle yaklaşır. Güç 0 iken FLOAT modunda
 * sürtünmeyle yavaşlar, BRAKE modunda hızla durur. Duvarlar robotu durdurur.
 */
public class MecanumPhysics {
    // Saha konumu
    private double x, y, heading;
    // Robot ekseninde hızlar: ileri (+x), sola (+y), saat yönü tersine dönüş
    private double vForward, vStrafe, omega;

    // Son verilen tekerlek güçleri (FL, FR, BL, BR) ve fren modu
    private final double[] wheelPowers = new double[4];
    private boolean brake = false;

    public MecanumPhysics(Pose start) {
        setPose(start);
    }

    public synchronized void setWheelPowers(double[] powers, boolean brake) {
        System.arraycopy(powers, 0, wheelPowers, 0, 4);
        this.brake = brake;
    }

    public synchronized void setPose(Pose pose) {
        x = pose.x();
        y = pose.y();
        heading = pose.heading();
    }

    /** Fiziği dt saniye ilerletir. */
    public synchronized void step(double dt) {
        double fl = wheelPowers[0], fr = wheelPowers[1], bl = wheelPowers[2], br = wheelPowers[3];

        // Mecanum ileri kinematiği (Pedro Mecanum'un karıştırmasının tersi):
        //   fl = f - s - t, fr = f + s + t, bl = f + s - t, br = f - s + t
        double f = (fl + fr + bl + br) / 4.0;
        double s = (-fl + fr + bl - br) / 4.0;
        double t = (-fl + fr - bl + br) / 4.0;

        boolean noPower = Math.abs(fl) < 1e-3 && Math.abs(fr) < 1e-3
                && Math.abs(bl) < 1e-3 && Math.abs(br) < 1e-3;

        if (noPower && !brake) {
            // FLOAT: sadece sürtünme
            vForward = towardZero(vForward, SimConfig.COAST_FORWARD_DECEL * dt);
            vStrafe = towardZero(vStrafe, SimConfig.COAST_STRAFE_DECEL * dt);
            omega = towardZero(omega, SimConfig.COAST_ANGULAR_DECEL * dt);
        } else {
            double tau = noPower ? SimConfig.BRAKE_TIME_CONSTANT : SimConfig.DRIVE_TIME_CONSTANT;
            double k = 1.0 - Math.exp(-dt / tau);
            vForward += (f * SimConfig.MAX_FORWARD_VELOCITY - vForward) * k;
            vStrafe += (s * SimConfig.MAX_STRAFE_VELOCITY - vStrafe) * k;
            omega += (t * SimConfig.MAX_ANGULAR_VELOCITY - omega) * k;
        }

        // Robot eksenindeki hızı sahaya çevirip entegre et (orta nokta açısı ile)
        double midHeading = heading + omega * dt / 2.0;
        double cos = Math.cos(midHeading), sin = Math.sin(midHeading);
        x += (vForward * cos - vStrafe * sin) * dt;
        y += (vForward * sin + vStrafe * cos) * dt;
        heading = normalize(heading + omega * dt);

        // Duvarlar
        double min = SimConfig.ROBOT_HALF_SIZE, max = SimConfig.FIELD_SIZE - SimConfig.ROBOT_HALF_SIZE;
        if (x < min || x > max || y < min || y > max) {
            x = clamp(x, min, max);
            y = clamp(y, min, max);
            // Duvara çarpınca duvara doğru olan hız bileşenini sıfırla
            double[] field = fieldVelocity();
            double vxField = field[0], vyField = field[1];
            if ((x <= min && vxField < 0) || (x >= max && vxField > 0)) vxField = 0;
            if ((y <= min && vyField < 0) || (y >= max && vyField > 0)) vyField = 0;
            vForward = vxField * Math.cos(heading) + vyField * Math.sin(heading);
            vStrafe = -vxField * Math.sin(heading) + vyField * Math.cos(heading);
        }
    }

    public synchronized Pose pose() {
        return new Pose(x, y, heading);
    }

    /** Saha eksenindeki hız: {vx, vy, omega} (Pinpoint'in getVelX/getVelY'si gibi). */
    public synchronized double[] fieldVelocity() {
        double cos = Math.cos(heading), sin = Math.sin(heading);
        return new double[]{
                vForward * cos - vStrafe * sin,
                vForward * sin + vStrafe * cos,
                omega
        };
    }

    public synchronized double[] wheelPowers() {
        return wheelPowers.clone();
    }

    private static double towardZero(double v, double amount) {
        if (Math.abs(v) <= amount) return 0.0;
        return v - Math.signum(v) * amount;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    static double normalize(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
