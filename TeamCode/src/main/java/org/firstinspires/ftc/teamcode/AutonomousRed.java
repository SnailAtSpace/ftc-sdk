package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.jar.Attributes;

@Autonomous(name = "Auto RED")
public class AutonomousRed extends CommonOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //Initialize(hardwareMap,true);
        Pose2d startPose = new Pose2d(7.25,-70.5+8.875, Math.toRadians(270));
        TrajectorySequence sequence = drive.trajectorySequenceBuilder(startPose)
                .back(2)
                .waitSeconds(0.1)
                .lineToConstantHeading(new Vector2d(-13,-60))
                .splineToConstantHeading(new Vector2d(-13.5,-35),90)
                .build();
        drive.setPoseEstimate(startPose);
        waitForStart();
        drive.followTrajectorySequence(sequence);
    }
}
