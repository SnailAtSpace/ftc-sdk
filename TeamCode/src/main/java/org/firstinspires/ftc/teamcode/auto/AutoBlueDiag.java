package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public abstract class AutoBlueDiag extends AutonomousBoilerplate{

    @Override
    public void runOpMode(boolean mirroredX, boolean mirroredY) throws InterruptedException {
        super.runOpMode(mirroredX, mirroredY);
    }

    public TrajectorySequence pathToFirstJunction(){
        return drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(Math.min(1400, DriveConstants.MAX_VEL), DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(startPose.plus(new Pose2d(5,-5,0)).vec())
                .splineToConstantHeading(new Vector2d(-400,fieldHalf-500),3*pi/2.0f)
                .splineToSplineHeading(junctionPose.plus(new Pose2d(5,-5,0)),Math.toRadians(-45))
                .UNSTABLE_addTemporalMarkerOffset(-1.8, ()->{
                    riserMotor.setTargetPosition(armExtensionToEncoderTicks(920));
                    riserMotor.setPower(1);
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .build();
    }

    public TrajectorySequence pathToConeLine(){
        return drive.trajectorySequenceBuilder(junctionPose)
                .lineToSplineHeading(new Pose2d(-300, 900, Math.toRadians(-75)))
                .splineToSplineHeading(new Pose2d(-320,600,3*pi/2.0f),3*pi/2.0f)
                .splineToSplineHeading(new Pose2d(-600,310,1.045*pi),pi)
                .splineToConstantHeading(coneLinePose.plus(new Pose2d(0,0,0)).vec(),1.25*pi)
                .build();
    }

    public TrajectorySequence pathToConesFromSecondJunction(){
        return drive.trajectorySequenceBuilder(secondJunctionPose)
                .lineTo(secondJunctionPose.plus(new Pose2d(-5,5,0)).vec())
                .splineToSplineHeading(coneLinePose.plus(new Pose2d(0,0,0)),1.1*pi)
                .build();
    }


    public TrajectorySequence pathToCones(){
        return drive.trajectorySequenceBuilder(new Pose2d(-1400,285,pi))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(Math.min(700,DriveConstants.MAX_VEL), DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(conePose.plus(new Pose2d(0,0,0)).vec(), pi)
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->riserServo.setPosition(1))
                .build();
    }

    public TrajectorySequence pathToSecondJunction(){
        return drive.trajectorySequenceBuilder(conePose)
                .lineTo(new Vector2d(-1200,350))
                .splineToSplineHeading(secondJunctionPose.plus(new Pose2d(0,0,0)), -0.1*pi)
                .UNSTABLE_addTemporalMarkerOffset(-1.175,()->{
                    riserMotor.setTargetPosition(armExtensionToEncoderTicks(940));
                    riserMotor.setPower(1);
                })
                .build();
    }

    public TrajectorySequence pathToParking1(){
        return drive.trajectorySequenceBuilder(secondJunctionPose)
                .back(20)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    riserMotor.setTargetPosition(0);
                    riserMotor.setPower(0.65);
                })
                .splineToSplineHeading(closeParkPose.plus(new Pose2d(0,0,0)),0)
                .build();
    }

    public TrajectorySequence pathToParking2(){
        return drive.trajectorySequenceBuilder(secondJunctionPose)
                .back(20)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    riserMotor.setTargetPosition(0);
                    riserMotor.setPower(0.65);
                })
                .splineToSplineHeading(centerParkPose.plus(new Pose2d(0,0,0)),0.5*pi)
                .build();
    }

    public TrajectorySequence pathToParking3(){
        return drive.trajectorySequenceBuilder(secondJunctionPose)
                .back(20)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    riserMotor.setTargetPosition(0);
                    riserMotor.setPower(0.65);
                })
                .splineToSplineHeading(edgeParkPose.plus(new Pose2d(0,0,0)),1*pi)
                .build();
    }
}
