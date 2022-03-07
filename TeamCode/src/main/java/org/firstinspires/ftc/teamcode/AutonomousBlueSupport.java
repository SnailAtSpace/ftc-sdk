package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Auto BLU: Support",preselectTeleOp = "1000-7?")
public class AutonomousBlueSupport extends CommonOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Initialize(hardwareMap,true);
        riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pose2d startposeSupportBlue = new Pose2d(-31.5,fieldHalf-hLength,Math.toRadians(90));
        TrajectorySequence preloadSequence = drive.trajectorySequenceBuilder(startposeSupportBlue)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-61,36),Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-33,24, Math.toRadians(0)),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(duckPos== BingusPipeline.RandomizationFactor.LEFT) {
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
                .back(0.5)
                .splineToConstantHeading(new Vector2d(-61,36),Math.toRadians(90))
                .build();
        drive.setPoseEstimate(startposeSupportBlue);
        riserMotor.setTargetPosition(0);
        while(!opModeIsActive() && !isStopRequested()){
            duckPos = pipeline.ComposeTelemetry(telemetry);
        }
        waitForStart();
        drive.followTrajectorySequence(preloadSequence);
        while (opModeIsActive()) {
            idle();
        }
    }
}

