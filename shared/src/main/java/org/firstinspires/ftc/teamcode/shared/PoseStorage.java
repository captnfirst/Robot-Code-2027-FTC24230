package org.firstinspires.ftc.teamcode.shared;

import com.pedropathing.math.Pose;

/**
 * Otonom bittiginde robotun konumunu teleop'a tasimak icin.
 * Otonom her dongude lastPose'u gunceller, teleop init'te buradan baslar.
 */
public final class PoseStorage {
    private PoseStorage() {}

    public static Pose lastPose = AutoPaths.START;
}
