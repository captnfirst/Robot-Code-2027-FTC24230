package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.utils.Utils;

import java.util.HashMap;
import java.util.Map;

/**
 * Pedro'nun gerçek Mecanum sınıfının simülasyon karşılığı.
 * Güç karıştırma ve normalizasyon com.pedropathing.revhub.drivetrains.Mecanum ile BİREBİR aynıdır;
 * tek fark motorlara değil fizik modeline yazmasıdır.
 */
public class SimDrivetrain implements Drivetrain {
    private final MecanumPhysics physics;
    private final double[] wheelPowers = new double[4];
    private DrivePowers lastPowers = DrivePowers.zero();
    private double powerScale = 1.0;

    public SimDrivetrain(MecanumPhysics physics) {
        this.physics = physics;
    }

    @Override
    public void drive(DrivePowers powers, boolean manual) {
        boolean brake = manual && SimConfig.MANUAL_BRAKE_MODE;
        lastPowers = powers;
        double[] raw = computeWheelPowersUnnormalized(powers);
        double maxPower = 1.0;
        for (double p : raw) maxPower = Math.max(maxPower, Math.abs(p));
        powerScale = 1.0 / maxPower;
        for (int i = 0; i < 4; i++) wheelPowers[i] = raw[i] / maxPower;
        physics.setWheelPowers(wheelPowers, brake);
    }

    static double[] computeWheelPowersUnnormalized(DrivePowers p) {
        double f = p.forward(), s = p.strafe(), t = p.turn();
        return new double[]{f - s - t, f + s + t, f + s - t, f - s + t};
    }

    @Override
    public double maxScaling(DrivePowers current, DrivePowers delta) {
        double lambda = 1.0;
        double[] a = computeWheelPowersUnnormalized(current);
        double[] b = computeWheelPowersUnnormalized(delta);
        for (int i = 0; i < 4; i++) {
            if (Math.abs(b[i]) < 1e-9) continue;
            double t1 = (1.0 - a[i]) / b[i];
            double t2 = (-1.0 - a[i]) / b[i];
            if (t1 >= 0.0 && t1 < lambda) lambda = t1;
            if (t2 >= 0.0 && t2 < lambda) lambda = t2;
        }
        return Utils.clamp(lambda, 0.0, 1.0);
    }

    @Override
    public void stop() {
        stop(SimConfig.MANUAL_BRAKE_MODE);
    }

    @Override
    public void stop(boolean brake) {
        java.util.Arrays.fill(wheelPowers, 0.0);
        lastPowers = DrivePowers.zero();
        physics.setWheelPowers(wheelPowers, brake);
    }

    @Override
    public Map<String, Object> debug() {
        Map<String, Object> map = new HashMap<>();
        map.put("forward", lastPowers.forward());
        map.put("strafe", lastPowers.strafe());
        map.put("turn", lastPowers.turn());
        map.put("powerScale", powerScale);
        return map;
    }

    @Override
    public double interpolateVelocity(double xRadius, double yRadius, double theta) {
        return 1.0 / (Math.abs(Math.cos(theta)) / xRadius + Math.abs(Math.sin(theta)) / yRadius);
    }

    public DrivePowers lastPowers() {
        return lastPowers;
    }
}
