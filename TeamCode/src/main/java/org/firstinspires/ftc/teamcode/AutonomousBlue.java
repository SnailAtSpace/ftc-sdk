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

@Autonomous(name = "Auto BLU",preselectTeleOp = "1000-7?")
public class AutonomousBlue extends CommonOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Initialize(hardwareMap,true);
        riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TrajectorySequence preloadSequence = drive.trajectorySequenceBuilder(startPoseBlue)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-12,41.5),Math.toRadians(180))
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
                .splineToSplineHeading(new Pose2d(7.5,fieldHalf-hDiag, Math.toRadians(0)),Math.toRadians(0))
                .splineToConstantHeading(defaultPoseBlue.vec(),Math.toRadians(90))
                .build();
        TrajectorySequence freightSequence = drive.trajectorySequenceBuilder(defaultPoseBlue)
                .setReversed(true)
                .lineTo(new Vector2d(fieldHalf-hLength-15,fieldHalf-hWidth))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{
                    collectorMotor.setPower(maxCollPower);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.25,()->{
                    collectorMotor.setPower(-maxCollPower);
                })
                .UNSTABLE_addTemporalMarkerOffset(2,()->{
                    collectorMotor.setPower(0);
                })
                .lineTo(defaultPoseBlue.vec())
                .build();
        TrajectorySequence deliverSequence = drive.trajectorySequenceBuilder(defaultPoseBlue)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(7.5,fieldHalf-hDiag),Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-12,41.5, Math.toRadians(90)),Math.toRadians(270))
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
                .splineToSplineHeading(new Pose2d(7.5,fieldHalf-hDiag, Math.toRadians(0)),Math.toRadians(0))
                .splineToConstantHeading(defaultPoseBlue.vec(),Math.toRadians(90))
                .build();
        TrajectorySequence parkSequence = drive.trajectorySequenceBuilder(defaultPoseBlue)
                .setReversed(true)
                .lineTo(new Vector2d(fieldHalf-hLength-23,fieldHalf-hWidth))
                .splineToConstantHeading(new Vector2d(fieldHalf-hLength-23,+fieldHalf-hDiag-0.5),270)
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(fieldHalf-23-hLength,+fieldHalf-hWidth-20))
                .build();
        drive.setPoseEstimate(startPoseBlue);
        riserMotor.setTargetPosition(0);
        while(!opModeIsActive() && !isStopRequested()){
            duckPos = pipeline.ComposeTelemetry(telemetry);
        }
        waitForStart();
        drive.followTrajectorySequence(preloadSequence);
        ramIntoWall(false);
//        drive.setPoseEstimate(defaultPoseBlue);
//        drive.followTrajectorySequence(freightSequence);
//        drive.setPoseEstimate(defaultPoseBlue);
//        drive.followTrajectorySequence(deliverSequence);
//        ramIntoWall(false);
        drive.setPoseEstimate(defaultPoseBlue);
        drive.followTrajectorySequence(parkSequence);
        while (opModeIsActive()) {
            idle();
        }
    }
}

