package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous
public class resetWorm extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotor Worm = hardwareMap.get(DcMotor.class, "Wmotor");
        waitForStart();
        if(opModeIsActive()) {
            Worm.setPower(1);
            sleep(2100);
            Worm.setPower(0);
        }
    }
}
