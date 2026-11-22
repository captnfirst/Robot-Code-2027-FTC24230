package org.firstinspires.ftc.teamcode.shared;

import com.pedropathing.follower.Follower;

/**
 * Sürüş mantığı. Robotta da simülasyonda da BİREBİR aynı kod çalışır.
 *
 * Kontroller:
 *   Sol çubuk      : ileri/geri + yanlara
 *   Sağ çubuk X    : dönüş
 *   Sol bumper     : yavaş mod (basılı tutarken)
 *   Back           : alan merkezli (field-centric) aç/kapa
 *   Start          : field-centric için "ileri" yönünü şu anki yön yap
 */
public class TeleopDrive {
    public static double DEADBAND = 0.05;
    public static double SLOW_MODE_SCALE = 0.35;

    private boolean fieldCentric = true;
    private double headingOffset = 0.0;

    public void update(Follower follower, DriverInput in) {
        if (in.backPressed()) fieldCentric = !fieldCentric;
        if (in.startPressed()) headingOffset = follower.pose().heading();

        // FTC'de çubuk yukarı = negatif; Pedro'da ileri = pozitif, sola = pozitif, sola dönüş = pozitif.
        double forward = -deadband(in.leftStickY);
        double strafe = -deadband(in.leftStickX);
        double turn = -deadband(in.rightStickX);

        if (in.leftBumper) {
            forward *= SLOW_MODE_SCALE;
            strafe *= SLOW_MODE_SCALE;
            turn *= SLOW_MODE_SCALE;
        }

        if (fieldCentric) {
            // Sürücünün "ileri"si sahada sabit kalsın: komutu robot açısının tersine döndür.
            double angle = -(follower.pose().heading() - headingOffset);
            double cos = Math.cos(angle), sin = Math.sin(angle);
            double f = forward * cos - strafe * sin;
            double s = forward * sin + strafe * cos;
            forward = f;
            strafe = s;
        }

        follower.manual(forward, strafe, turn);
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }

    private static double deadband(double v) {
        return Math.abs(v) < DEADBAND ? 0.0 : v;
    }
}
