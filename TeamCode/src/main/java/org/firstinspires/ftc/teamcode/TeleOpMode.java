package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public abstract class TeleOpMode extends CommonOpMode {
    @Override
    public void Initialize(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        super.Initialize(hardwareMap);
        riserMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static double logifyInput(double input, double power) {
        return Math.pow(Math.abs(input), power) * Math.signum(input);
    }
}
