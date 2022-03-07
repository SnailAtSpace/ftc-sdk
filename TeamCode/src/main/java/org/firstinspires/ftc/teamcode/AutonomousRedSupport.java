package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auto RED Pos5",preselectTeleOp = "1000-7?")
public class AutonomousRedSupport extends CommonOpMode {
    @Override
    public void runOpMode() {
        Initialize(hardwareMap, true);
        this.riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
    }
}

