package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "Toyota Mark II Simulation")
public class AutoNothing extends CommonOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        while(opModeInInit()){}
        while(opModeIsActive() && !isStopRequested()){}
    }
}
