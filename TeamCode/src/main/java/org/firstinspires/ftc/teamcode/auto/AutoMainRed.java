package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "Toyota Mark II Simulation", name = "MAIN RED")
public class AutoMainRed extends AutonomousBoilerplate{

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(true, false);
    }
}
