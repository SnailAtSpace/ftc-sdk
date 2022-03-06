package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@Deprecated
@TeleOp
public class ResetArmEncoder extends CommonOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor riserMotor = hardwareMap.get(DcMotor.class, "riserMotor");
        telemetry.addLine("Waiting...");
        telemetry.update();
        waitForStart();
        riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        riserMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addLine("Arm set to 0 in this position.");
        telemetry.update();
    }
}
