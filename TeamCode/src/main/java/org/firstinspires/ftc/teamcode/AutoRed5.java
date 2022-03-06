package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Disabled
@Deprecated
@Autonomous(name = "Dota 2 Fifth Position", preselectTeleOp = "1000-7?")
public class AutoRed5 extends CommonOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Initialize(hardwareMap,true);
        riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pose2d startposeSupportRed = new Pose2d(-39.5,-fieldHalf+hLength,Math.toRadians(270));
        TrajectorySequence loadSequence = drive.trajectorySequenceBuilder(startposeSupportRed)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-55,-36),Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-30,-24, Math.toRadians(180)),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(duckPos == BingusPipeline.RandomizationFactor.LEFT) {
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
                .UNSTABLE_addTemporalMarkerOffset(1.75,()-> freightServo.setPosition(0))
                .UNSTABLE_addTemporalMarkerOffset(3.25,()-> freightServo.setPosition(1))
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
                .back(0.5)
                .splineToSplineHeading(new Pose2d(-61,-36,Math.toRadians(270)),Math.toRadians(270))
                .lineTo(new Vector2d(-61,-fieldHalf+7.5+hLength))
                .build();
        TrajectorySequence carouselSequence = drive.trajectorySequenceBuilder(new Pose2d(-fieldHalf+hWidth,-fieldHalf+225/25.4+hLength,Math.toRadians(270)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-60,-36,Math.toRadians(270)),Math.toRadians(270))
                .build();

        drive.setPoseEstimate(startposeSupportRed);
        riserMotor.setTargetPosition(0);
        while(!opModeIsActive() && !isStopRequested()){
            duckPos = pipeline.ComposeTelemetry(telemetry);
        }

        waitForStart();
        drive.followTrajectorySequence(loadSequence);
        carouselMotor.setPower(-0.25);
        safeSleep(2500);
        carouselMotor.setPower(0);
        drive.followTrajectorySequence(carouselSequence);
        while (opModeIsActive()) {
            idle();
        }
    }
}
