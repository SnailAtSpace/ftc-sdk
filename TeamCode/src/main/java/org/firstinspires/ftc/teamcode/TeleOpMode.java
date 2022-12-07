package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class TeleOpMode extends CommonOpMode {
    @Override
    public void Initialize(HardwareMap hardwareMap) {
        super.Initialize(hardwareMap);

        riserMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static double logifyInput(double input, double power) {
        return Math.pow(Math.abs(input), power) * Math.signum(input);
    }
}
