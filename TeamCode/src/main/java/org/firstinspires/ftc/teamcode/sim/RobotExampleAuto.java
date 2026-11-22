package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.shared.AutoPaths;
import org.firstinspires.ftc.teamcode.shared.PoseStorage;

import java.util.List;

/**
 * Simülasyonda izlediğin otonomun robottaki hâli (shared/AutoPaths.java).
 * DİKKAT: Robotu sahada AutoPaths.START konumuna ve yönüne koymadan çalıştırma.
 */
@Autonomous(name = "Ornek Otonom (Sim ile ayni)", group = "Sim")
public class RobotExampleAuto extends OpMode {
    private Follower follower;
    private List<Path> paths;
    private int index = 0;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(AutoPaths.START);
        paths = AutoPaths.exampleAuto();
    }

    @Override
    public void start() {
        index = 0;
        follower.follow(paths.get(0));
    }

    @Override
    public void loop() {
        if (index < paths.size() && !follower.isBusy()) {
            index++;
            if (index < paths.size()) follower.follow(paths.get(index));
        }
        follower.update();

        Pose p = follower.pose();
        PoseStorage.lastPose = p;
        telemetry.addData("Path", "%d / %d", Math.min(index + 1, paths.size()), paths.size());
        telemetry.addData("x (in)", "%.1f", p.x());
        telemetry.addData("y (in)", "%.1f", p.y());
        telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(p.heading()));
        telemetry.update();
    }
}
