package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.shared.DriverInput;
import org.firstinspires.ftc.teamcode.shared.TeleopDrive;
import org.firstinspires.ftc.teamcode.shared.PoseStorage;

/**
 * Simülasyonda test ettiğin sürüşün robottaki hâli.
 * Sürüş mantığı shared/TeleopDrive.java'dadır; simülatör de aynı sınıfı çalıştırır.
 */
@TeleOp(name = "Drive (Sim ile ayni)", group = "Sim")
public class RobotDriveTeleop extends OpMode {
    private Follower follower;
    private final TeleopDrive drive = new TeleopDrive();
    private final DriverInput input = new DriverInput();

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(PoseStorage.lastPose);
    }

    @Override
    public void loop() {
        copyGamepad(gamepad1, input);
        drive.update(follower, input);
        follower.update();
        input.endCycle();

        Pose p = follower.pose();
        telemetry.addData("x (in)", "%.1f", p.x());
        telemetry.addData("y (in)", "%.1f", p.y());
        telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(p.heading()));
        telemetry.addData("Field-centric", drive.isFieldCentric());
        telemetry.update();
    }

    /** FTC Gamepad -> donanımdan bağımsız DriverInput. */
    public static void copyGamepad(Gamepad g, DriverInput in) {
        in.leftStickX = g.left_stick_x;
        in.leftStickY = g.left_stick_y;
        in.rightStickX = g.right_stick_x;
        in.rightStickY = g.right_stick_y;
        in.leftTrigger = g.left_trigger;
        in.rightTrigger = g.right_trigger;
        in.a = g.a;
        in.b = g.b;
        in.x = g.x;
        in.y = g.y;
        in.leftBumper = g.left_bumper;
        in.rightBumper = g.right_bumper;
        in.dpadUp = g.dpad_up;
        in.dpadDown = g.dpad_down;
        in.dpadLeft = g.dpad_left;
        in.dpadRight = g.dpad_right;
        in.start = g.start;
        in.back = g.back;
    }
}
