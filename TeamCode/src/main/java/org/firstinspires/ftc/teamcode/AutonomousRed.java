package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.jar.Attributes;

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
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(duckPos==BingusPipeline.RandomizationFactor.LEFT) {
                        riserMotor.setTargetPosition(500);
                    }
                    else if(duckPos == BingusPipeline.RandomizationFactor.CENTER){
                        riserMotor.setTargetPosition(750);
                    }
                    else if(duckPos == BingusPipeline.RandomizationFactor.RIGHT) riserMotor.setTargetPosition(1100);
                    riserMotor.setVelocity(400, AngleUnit.DEGREES);
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    riserMotor.setVelocity(200, AngleUnit.DEGREES);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.75,()->{
                    freightServo.setPosition(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.25,()->{
                    freightServo.setPosition(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.75,()->{
                    riserMotor.setTargetPosition(10);
                    riserMotor.setVelocity(150, AngleUnit.DEGREES);
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    riserMotor.setVelocity(150, AngleUnit.DEGREES);
                })
                .UNSTABLE_addTemporalMarkerOffset(4.5,()->{
                    riserMotor.setTargetPosition(20);
                    riserMotor.setVelocity(200, AngleUnit.DEGREES);
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    riserMotor.setVelocity(200, AngleUnit.DEGREES);
                })
                .waitSeconds(4.5)
                .forward(0.5)
                .splineToSplineHeading(new Pose2d(7.5,-fieldHalf+hDiag, Math.toRadians(180)),Math.toRadians(270))
                .splineToConstantHeading(defaultPoseRed.vec(),Math.toRadians(270))
                .build();
        TrajectorySequence freightSequence = drive.trajectorySequenceBuilder(defaultPoseRed)
                .setReversed(true)
                .lineTo(new Vector2d(fieldHalf-hLength-15,-fieldHalf+hWidth))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{
                    collectorMotor.setPower(maxCollPower);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.25,()->{
                    collectorMotor.setPower(-maxCollPower);
                })
                .UNSTABLE_addTemporalMarkerOffset(2,()->{
                    collectorMotor.setPower(0);
                })
                .lineTo(defaultPoseRed.vec())
                .build();
        TrajectorySequence deliverSequence = drive.trajectorySequenceBuilder(defaultPoseRed)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(7.5,-fieldHalf+hDiag),Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-12,-41.5, Math.toRadians(270)),Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    riserMotor.setTargetPosition(1000);
                    riserMotor.setVelocity(400, AngleUnit.DEGREES);
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    riserMotor.setVelocity(200, AngleUnit.DEGREES);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.75,()->{
                    freightServo.setPosition(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.25,()->{
                    freightServo.setPosition(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.75,()->{
                    riserMotor.setTargetPosition(0);
                    riserMotor.setVelocity(200, AngleUnit.DEGREES);
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    riserMotor.setVelocity(200, AngleUnit.DEGREES);
                })
                .UNSTABLE_addTemporalMarkerOffset(4.5,()->{
                    riserMotor.setTargetPosition(20);
                    riserMotor.setVelocity(200, AngleUnit.DEGREES);
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    riserMotor.setVelocity(200, AngleUnit.DEGREES);
                })
                .waitSeconds(4.5)
                .forward(0.5)
                .splineToSplineHeading(new Pose2d(7.5,-fieldHalf+hDiag, Math.toRadians(180)),Math.toRadians(170))
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
//        drive.setPoseEstimate(defaultPoseRed);
//        drive.followTrajectorySequence(freightSequence);
//        drive.setPoseEstimate(defaultPoseRed);
//        drive.followTrajectorySequence(deliverSequence);
//        ramIntoWall(true);
        drive.setPoseEstimate(defaultPoseRed);
        drive.followTrajectorySequence(parkSequence);
        while (opModeIsActive()) {
            idle();
        }
    }
}
