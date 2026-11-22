package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import java.util.Random;

/**
 * goBILDA Pinpoint'in simülasyon karşılığı. Pedro'nun PinpointLocalizer'ı gibi
 * saha ekseninde konum ve hız döndürür; değerleri fizik modelinden okur.
 */
public class SimLocalizer implements Localizer {
    private final MecanumPhysics physics;
    private final Random random = new Random(6459);
    private MotionState state = MotionState.zero();

    public SimLocalizer(MecanumPhysics physics) {
        this.physics = physics;
        update();
    }

    @Override
    public void setPose(Pose pose) {
        physics.setPose(pose);
        state = state.withPose(pose);
    }

    @Override
    public void update() {
        Pose p = physics.pose();
        if (SimConfig.POSE_NOISE_INCHES > 0 || SimConfig.HEADING_NOISE_RAD > 0) {
            p = new Pose(
                    p.x() + random.nextGaussian() * SimConfig.POSE_NOISE_INCHES,
                    p.y() + random.nextGaussian() * SimConfig.POSE_NOISE_INCHES,
                    p.heading() + random.nextGaussian() * SimConfig.HEADING_NOISE_RAD);
        }
        double[] v = physics.fieldVelocity();
        state = MotionState.ofVelocity(p, new Velocity(v[0], v[1], v[2]));
    }

    @Override
    public MotionState state() {
        return state;
    }

    @Override
    public void reset() {
        // Gerçek Pinpoint'te IMU kalibrasyonu yapılır; simülasyonda gerek yok.
    }
}
