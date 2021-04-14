package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous
public class resetWorm extends CommonOpMode {
    @Override
    public void runOpMode(){
        DcMotor Worm = hardwareMap.get(DcMotor.class, "Wmotor");
        Worm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        if(opModeIsActive()) {
            RetractArm();
        }
    }
}
