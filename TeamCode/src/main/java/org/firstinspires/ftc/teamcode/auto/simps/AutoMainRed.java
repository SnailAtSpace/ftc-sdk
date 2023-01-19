package org.firstinspires.ftc.teamcode.auto.simps;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoRedDiag;

@Autonomous(preselectTeleOp = "Toyota Mark II Simulation", name = "MAIN RED")
public class AutoMainRed extends AutoRedDiag {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(true, false);
    }
}
