package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class TeleOpMode extends CommonOpMode {
    @Override
    public void Initialize(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        super.Initialize(hardwareMap);
        riserMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collectorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static double logifyInput(double input, double power) {
        return Math.pow(Math.abs(input), power) * Math.signum(input);
    }
}
