package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class DistanceTest extends CommonOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Initialize(hardwareMap);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            telemetry.addData("dist: ", distanceSensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
