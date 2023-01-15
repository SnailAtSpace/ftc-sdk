package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "Toyota Mark II Simulation", name = "MAIN BLUE")
public class AutoSupportBlue extends AutonomousBoilerplate{

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(false, true);
    }
}
