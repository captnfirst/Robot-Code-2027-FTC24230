package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class Superstructure {
    public static final Flywheel flywheel = new Flywheel();
    public static final Feeder feeder = new Feeder();

    public static double RPM = 3500.0;
    private static boolean isShooting = false;
    public static boolean flywheelIdleMode = true;
    public static double flywheelIdleRPM = 2800.0;

    public static void init(HardwareMap hardwareMap) {
        flywheel.init(hardwareMap);
        feeder.init(hardwareMap);
    }

    public static void periodic() {
        if (flywheelIdleMode && !isShooting) {
            flywheel.setSetpointRPM(flywheelIdleRPM);
        }

        flywheel.periodic();
        feeder.periodic();
    }

    public static void read() {
        flywheel.read();
        feeder.read();
    }

    public static void write() {
        flywheel.write();
        feeder.write();
    }

    public static void reset() {
        flywheel.reset();
        feeder.reset();
        flywheelIdleMode = true;
        isShooting = false;
    }

    public static void setShootSystem(){
        isShooting = true;
        if(flywheel.IsAtSetpoint()){
            feeder.setFeederMotor(1.0);
        }else{
            feeder.setFeederMotor(0.0);
        }
        flywheel.setSetpointRPM(RPM);
    }

    public static void stopShooting(){
        isShooting = false;
        flywheel.setSetpointRPM(0.0);
        feeder.setFeederMotor(0.0);
    }
}