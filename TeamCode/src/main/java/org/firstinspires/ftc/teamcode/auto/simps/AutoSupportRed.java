package org.firstinspires.ftc.teamcode.auto.simps;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoBlueDiag;

@Autonomous(preselectTeleOp = "Toyota Mark II Simulation", name = "SUPPORT RED")
public class AutoSupportRed extends AutoBlueDiag {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(true, true);
    }
}
