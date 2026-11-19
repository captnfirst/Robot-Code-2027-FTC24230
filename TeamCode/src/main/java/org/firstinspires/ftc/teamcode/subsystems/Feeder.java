package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

public class Feeder extends WSubsystem {

    DcMotorEx feederMotor;

    @Override
    public void init(HardwareMap hardwareMap) {
        feederMotor = hardwareMap.get(DcMotorEx.class,"feederMotor");
        feederMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        feederMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setFeederMotor(double speed){
        feederMotor.setPower(speed);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void read() {

    }

    @Override
    public void write() {

    }

    @Override
    public void reset() {

    }
}
