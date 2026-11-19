package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.wrappers.WVelocityGroup;


// @Config
public class Flywheel extends WSubsystem {
//    public static double ks=0;
//    public static double kv=0.002;
//    public static double kp=0;
    DcMotorEx masterShooterMotor;
    DcMotorEx slaveShooterMotor;
    WVelocityGroup VelocityController;
    WEncoder encoder;
    @Override
    public void init(HardwareMap hardwareMap) {
        masterShooterMotor = hardwareMap.get(DcMotorEx.class,"masterShooterMotor");
        slaveShooterMotor = hardwareMap.get(DcMotorEx.class,"slaveShooterMotor");

        masterShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slaveShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        masterShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slaveShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        encoder = new WEncoder(new MotorEx(hardwareMap, "masterShooterMotor").encoder);

        VelocityController = new WVelocityGroup(encoder,masterShooterMotor,slaveShooterMotor)
                .setPIDController(new PIDController(0.03,0.0,0.0))
                .setFeedforwardSimple(0.000366,0.0024,0.0);
    }

    public boolean IsAtSetpoint(){
        return Util.epsilonEquals(getShooterRPM(),VelocityController.getTargetVelocity(),100);
    }

    public void setSetpointRPM(double rpm){
        VelocityController.setTargetVelocity(rpm);
    }

    public double getShooterRPM(){
        return VelocityController.getVelocity();
    }

    public double getTarget() { return VelocityController.getTargetVelocity(); }

    @Override
    public void periodic() {
        //VelocityController.setFeedforwardSimple(ks,kv,0.0);
        //VelocityController.setPID(kp,0.0,0.0);
        VelocityController.periodic();
    }

    @Override
    public void read() {
        VelocityController.read();
    }

    @Override
    public void write() {
        VelocityController.write();
    }

    @Override
    public void reset() {

    }
}
