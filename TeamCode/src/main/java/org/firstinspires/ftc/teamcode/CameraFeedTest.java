package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.internal.opengl.models.Teapot;

@Autonomous(name = "Feed")
public class CameraFeedTest extends CommonOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Initialize(hardwareMap,true);
        waitForStart();
        while(opModeIsActive()){
            telemetry.addLine("Camera active.");
        }
    }
}
