package org.firstinspires.ftc.teamcode.red.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Superstructure;

@TeleOp(name = "shootFeedDeneme")
public class redTeleop extends OpMode {
    private Telemetry tel;

    @Override
    public void init() {
        tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Superstructure.init(hardwareMap);
        Superstructure.reset();
    }

    @Override
    public void loop() {
        if(gamepad1.right_bumper){
            Superstructure.setShootSystem();
        } else {
            Superstructure.stopShooting();
        }

        tel.addData("Controller hedef", Superstructure.flywheel.getTarget());
        tel.addData("Gerçek RPM", Superstructure.flywheel.getShooterRPM());
        telemetry.update();

        Superstructure.read();
        Superstructure.periodic();
        Superstructure.write();
    }
}