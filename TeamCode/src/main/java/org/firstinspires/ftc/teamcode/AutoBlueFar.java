package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Autonomous(name = "Blue mf (far)", preselectTeleOp = "Toyota Mark II Simulation")
public class AutoBlueFar extends CommonOpMode {
    // рандомизация -> опа ставим жёлтый на спот -> опа ставим фиолетовый на рандомизацию -> паркуемся

    private AprilTagProcessor apriltag;

    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        Initialize(hardwareMap);
        riserServoA.setPosition(0);
        riserServoB.setPosition(0);
        drive.pose = new Pose2d(-24 - sDist, 72 - rDist, 1.5 * Math.PI); // выравниваемся по **внешней** части
        telemetry.addLine("Hold on...");
        telemetry.update();
        sleep(1000);
        camUp();
        rand = 0;
        Action okipullup = drive.actionBuilder(drive.pose)
                .setTangent(1.5 * Math.PI)
                .splineToLinearHeading(new Pose2d(42, 36, 0), 0)
                .stopAndAdd(arm.changePos(500))
                .build();
        Action afterpartyL = drive.actionBuilder(drive.pose)
                .setTangent(1.5 * Math.PI)
                .splineToLinearHeading(new Pose2d(-24, 36, 0.5 * Math.PI), 0)
                .stopAndAdd(arm.dispenseViaCollector())
                .waitSeconds(1)
                .build();
        Action afterpartyC = drive.actionBuilder(drive.pose)
                .setTangent(1.5 * Math.PI)
                .splineToLinearHeading(new Pose2d(-32, 24 + rDist, 1.5 * Math.PI), 1.5 * Math.PI)
                .stopAndAdd(arm.dispenseViaCollector())
                .waitSeconds(1)
                .build();
        Action afterpartyR = drive.actionBuilder(drive.pose)
                .setTangent(1.5 * Math.PI)
                .splineToLinearHeading(new Pose2d(-48, 36, 0.5 * Math.PI), 0)
                .stopAndAdd(arm.dispenseViaCollector())
                .waitSeconds(1)
                .build();

        while (opModeInInit()) {
            rand = pipeline.ComposeTelemetry(telemetry);
            // 0;
        }
        //double what = 12 / drive.voltageSensor.getVoltage();
        // IT'S TIME TO RANCH IT UP
        webcam.closeCameraDeviceAsync(this::initAprilTag);

        Action iDrive = drive.actionBuilder(new Pose2d(42, 36, 0))
                .splineToConstantHeading(new Vector2d(72 - 11.25 + 4.75 * 0.577 - fDist,
                        36 - (rand - 1) * 6), 0)
                //.stopAndAdd(arm.changePusherState(true))
                .afterTime(0.5, arm.changePusherState(true))
                .afterTime(0.01, arm.changePusherState(false))
                .build();
        Action cyaLaterAlligator = drive.actionBuilder(new Pose2d(new Vector2d(hLength, 15), 0.5 * Math.PI))
                .splineToConstantHeading(new Vector2d(60, 6), 0)
                .build();

        Actions.runBlocking(okipullup);// to observation point
        safeSleep(1000);
        ArrayList<AprilTagDetection> detections = apriltag.getFreshDetections();
        for (AprilTagDetection d : detections) {
            if ((d.id - 1) % 3 == rand) { // rei-amayado what in the world
                double range2d = Math.hypot(d.ftcPose.x, d.ftcPose.y);
                drive.pose = new Pose2d(72 - 11.25 + 4.75 * 0.577 - range2d * Math.cos(d.ftcPose.yaw - d.ftcPose.bearing) - fDist * Math.cos(d.ftcPose.yaw),
                        36 - (rand - 1) * 6 - range2d * Math.sin(Math.abs(d.ftcPose.yaw - d.ftcPose.bearing) + fDist * Math.sin(d.ftcPose.yaw)),
                        -d.ftcPose.yaw);
            }
        }
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(arm.changeCassetteTilt(1), iDrive), // to backstage
                new ParallelAction(afterpartyL, arm.changePos(0), arm.changeCassetteTilt(0)), // to *the lines*
                new ParallelAction(cyaLaterAlligator, arm.changePos(0), arm.changeCassetteTilt(0), arm.changePusherState(false)) // parking
        ));
/*
        riserMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        riserMotor.setPower(-1);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.5*what, -what), 0)); //TODO
        safeSleep(1000);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
        safeSleep(800);
        riserMotor.setPower(0);
        riserServoA.setPosition(1);
        riserServoB.setPosition(1);
        drive.pose = new Pose2d(0,0,0.5*Math.PI);
        Actions.runBlocking(turn);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.5*what, 0.2*what*(rand-1)),0));
        safeSleep(1000);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
        safeSleep(100);
        pusherServo.setPosition(1);
        safeSleep(500);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-0.5*what, 0),0));
        safeSleep(100);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.1*what, what),0));
        safeSleep(600);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
*/
        while (opModeIsActive() && !isStopRequested()) {
            idle();
        }
    }

    private void initAprilTag() {
        apriltag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(apriltag)
                .build();
    }

}