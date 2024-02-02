package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Autonomous(name = "Test me!")
public class AutoSomething extends CommonOpMode{
    // TODO: определение, парковка
    // рандомизация -> опа ставим жёлтый на спот -> опа ставим фиолетовый на рандомизацию -> паркуемся

    private AprilTagProcessor apriltag;

    private VisionPortal visionPortal;
    @Override
    public void runOpMode() throws InterruptedException {
        Initialize(hardwareMap);
        initAprilTag();
        visionPortal.stopStreaming();
        telemetry.addLine("Hold on...");
        telemetry.update();
        sleep(1000);
        camUp();
        Action okipullup = drive.actionBuilder(drive.pose) //FIXME: replace drive.pose with the starting pose
                .splineToLinearHeading(new Pose2d(42, -36, 0), 0)
                .stopAndAdd(arm.changePos(300)) //TODO: check this for a possible sign error!
                .build();


        while(opModeInInit()){
            rand = pipeline.ComposeTelemetry(telemetry);
        }
        webcam.closeCameraDeviceAsync(() -> {
            // IT'S TIME TO RANCH IT UP
            visionPortal.resumeStreaming();
        });

        Action iDrive = drive.actionBuilder(new Pose2d(42, -36,0))
                .splineToLinearHeading(new Pose2d(72-11.25+3.25*0.577-hLength,
                        -36-(rand-1)*6, 0),0)
                .stopAndAdd(arm.changePusherState(true))
                .afterTime(0.5, arm.changePusherState(false))
                .build();

        Actions.runBlocking(okipullup);
        ArrayList<AprilTagDetection> detections = apriltag.getFreshDetections();
        for(AprilTagDetection d:detections){
            if((d.id-1)%3 == rand) // rei-amayado what in the world
                drive.pose = new Pose2d(72-11.25+3.25*0.577-d.ftcPose.range*Math.cos(d.ftcPose.yaw+d.ftcPose.bearing),
                                        -36-(rand-1)*6-d.ftcPose.range*Math.sin(d.ftcPose.yaw+d.ftcPose.bearing),
                                                -d.ftcPose.yaw); //FIXME (probably)
        }
        Actions.runBlocking(new ParallelAction(arm.changeCassetteTilt(1), iDrive));
    }

    private void initAprilTag(){
        apriltag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(apriltag)
                .build();
    }

}