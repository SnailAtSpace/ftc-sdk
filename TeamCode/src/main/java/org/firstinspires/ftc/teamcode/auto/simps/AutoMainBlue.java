package org.firstinspires.ftc.teamcode.auto.simps;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoBlueDiag;

@Autonomous(preselectTeleOp = "Toyota Mark II Simulation", name = "MAIN BLUE")
public class AutoMainBlue extends AutoBlueDiag {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(false, false);
    }
}
