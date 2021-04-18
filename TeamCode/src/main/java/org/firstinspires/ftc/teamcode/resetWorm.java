package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous
public class resetWorm extends CommonOpMode {
    @Override
    public void runOpMode(){
        Initialize(hardwareMap,false, BingusPipeline.StartLine.RIGHT);
        waitForStart();
        if(opModeIsActive()) {
            RetractArm();
        }
    }
}
