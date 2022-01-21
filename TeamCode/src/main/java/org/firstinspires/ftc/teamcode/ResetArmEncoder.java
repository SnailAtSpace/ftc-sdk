package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@TeleOp
public class ResetArmEncoder extends CommonOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Initialize(hardwareMap,false);
        riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        riserMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addLine("Arm set to 0 in this position.");
    }
}
