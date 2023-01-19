package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "DO NOTHING", preselectTeleOp = "Toyota Mark II Simulation")
public class AutoNothing extends AutoOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){}
    }
}
