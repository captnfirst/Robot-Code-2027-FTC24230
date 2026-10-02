package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

public class Vision extends WSubsystem {
    public Limelight3A LL3;
    public double tx;
    public double ty;
    public boolean tv;
    public double tl;
    LLResult results;

    @Override
    public void init(HardwareMap hardwareMap) {
        LL3 = hardwareMap.get(Limelight3A.class, "limelight");
        LL3.start();
    }

    @Override
    public void periodic() {}

    @Override
    public void read() {
        this.results = LL3.getLatestResult();
        if (results != null && results.isValid()) {
            this.tv = true;
            this.tx = results.getTx();
            this.ty = results.getTy();
            this.tl = results.getTargetingLatency() / 100.0;

            // Liste boş değilse ilk tag'e eriş
            if (!results.getFiducialResults().isEmpty()) {
                LLResultTypes.FiducialResult tag = results.getFiducialResults().get(0);
            }
        } else {
            this.tv = false;
            this.tx = 0.0;
            this.ty = 0.0;
        }
    }

    @Override
    public void write() {}

    @Override
    public void reset() {}
}