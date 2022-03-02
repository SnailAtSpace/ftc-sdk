package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.io.File;

@Autonomous(name = "Auto RED",preselectTeleOp = "1000-7?")
public class AutonomousRed extends CommonOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Initialize(hardwareMap,true);
        riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TrajectorySequence preloadSequence = drive.trajectorySequenceBuilder(startPoseRed)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-12.5,-41.5),Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    int tgtPos = 50;
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(duckPos==BingusPipeline.RandomizationFactor.LEFT) {
                        tgtPos = 250;
                    }
                    else if(duckPos == BingusPipeline.RandomizationFactor.CENTER){
                        tgtPos = 500;
                    }
                    else if(duckPos == BingusPipeline.RandomizationFactor.RIGHT) tgtPos = 1035;
                    riserMotor.setTargetPosition(tgtPos);
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    riserMotor.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1,()->{
                    freightServo.setPosition(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2,()->{
                    freightServo.setPosition(1);
                    riserMotor.setTargetPosition(0);
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    riserMotor.setPower(1);
                })
                .waitSeconds(2)
                .forward(0.5)
                .splineToSplineHeading(new Pose2d(7.5,-fieldHalf+hDiag, Math.toRadians(0)),Math.toRadians(270))
                .splineToConstantHeading(defaultPoseRed.vec(),Math.toRadians(270))
                .build();
        TrajectorySequence freightSequence = drive.trajectorySequenceBuilder(defaultPoseRed)
                .setReversed(true)
                .lineTo(new Vector2d(fieldHalf-hLength-15,-fieldHalf+hWidth))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{
                    collectorMotor.setPower(maxCollPower*0.85);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.25,()->{
                    collectorMotor.setPower(-maxCollPower*0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(2,()->{
                    collectorMotor.setPower(0);
                })
                .lineTo(defaultPoseRed.vec())
                .build();
        TrajectorySequence deliverSequence = drive.trajectorySequenceBuilder(defaultPoseRed)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(7.5,-fieldHalf+hDiag),Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-12.5,-41.5, Math.toRadians(270)),Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    riserMotor.setTargetPosition(500);
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    riserMotor.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{
                    freightServo.setPosition(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5,()->{
                    freightServo.setPosition(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(2,()->{
                    riserMotor.setTargetPosition(0);
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    riserMotor.setPower(1);
                })
                .waitSeconds(2)
                .forward(0.5)
                .splineToSplineHeading(new Pose2d(7.5,-fieldHalf+hDiag, Math.toRadians(0)),Math.toRadians(270))
                .splineToConstantHeading(defaultPoseRed.vec(),Math.toRadians(270))
                .build();
        TrajectorySequence parkSequence = drive.trajectorySequenceBuilder(defaultPoseRed)
                .setReversed(true)
                .lineTo(new Vector2d(fieldHalf-hLength-20,-fieldHalf+hWidth))
                .splineToConstantHeading(new Vector2d(fieldHalf-hLength-20,-fieldHalf+hDiag+0.5),90)
                .lineTo(new Vector2d(fieldHalf-20-hLength,-fieldHalf+hWidth+27.5))
                .build();
        drive.setPoseEstimate(startPoseRed);
        riserMotor.setTargetPosition(0);
        while(!opModeIsActive() && !isStopRequested()){
            duckPos = pipeline.ComposeTelemetry(telemetry);
        }
        waitForStart();
        drive.followTrajectorySequence(preloadSequence);
        ramIntoWall(true);
        drive.setPoseEstimate(defaultPoseRed);
        drive.followTrajectorySequence(freightSequence);
        drive.setPoseEstimate(defaultPoseRed);
        drive.followTrajectorySequence(deliverSequence);
        ramIntoWall(true);
        drive.setPoseEstimate(defaultPoseRed);
        drive.followTrajectorySequence(parkSequence);
        Pose2d lastPose = drive.getPoseEstimate();
        String filename = "LastAutoPosition";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, lastPose.getX()+" "+lastPose.getY()+" "+lastPose.getHeading());
        while (opModeIsActive()) {
            idle();
        }
    }
}
