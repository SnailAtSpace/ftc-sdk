package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ResetArm extends CommonOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        Initialize(hardwareMap,false);
        waitForStart();
        riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
