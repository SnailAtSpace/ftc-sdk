package org.firstinspires.ftc.teamcode.auto.simps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.AutoChill;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(preselectTeleOp = "Toyota Mark II Simulation", name = "chill blue")
public class AutoChillBlue extends AutoChill {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(false);
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

    public TrajectorySequence pathToParking1(){
        return drive.trajectorySequenceBuilder(junctionPose)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    riserMotor.setTargetPosition(0);
                    riserMotor.setPower(0.65);
                })
                .lineToConstantHeading(new Vector2d(-300, 900))
                .splineToSplineHeading(new Pose2d(-270,280,pi),3*pi/2.0f)
                .build();
    }

    public TrajectorySequence pathToParking2(){
        return drive.trajectorySequenceBuilder(junctionPose)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    riserMotor.setTargetPosition(0);
                    riserMotor.setPower(0.65);
                })
                .lineToConstantHeading(new Vector2d(-300, 900))
                .splineToSplineHeading(new Pose2d(-320,350,pi),1.25*pi)
                .splineToConstantHeading(new Vector2d(-850,290),pi)
                .build();
    }

    public TrajectorySequence pathToParking3(){
        return drive.trajectorySequenceBuilder(junctionPose)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    riserMotor.setTargetPosition(0);
                    riserMotor.setPower(0.65);
                })
                .lineToConstantHeading(new Vector2d(-300, 900))
                .splineToSplineHeading(new Pose2d(-320,350,pi),1.25*pi)
                .splineToConstantHeading(new Vector2d(-1400,250),pi)
                .build();
    }
}
