package org.firstinspires.ftc.teamcode.auto;


import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public abstract class AutonomousBoilerplate extends AutoOpMode{
    protected final Pose2d startPose = new Pose2d(-600,fieldHalf-hWidth,0);
    protected TrajectorySequence firstJunctionSequence;
    protected TrajectorySequence getConeSequence;
    protected TrajectorySequence nudgePathSequence;
    protected TrajectorySequence coneLineSequence;
    protected TrajectorySequence secondJunctionSequence;

    public void runOpMode(boolean mirrored) throws InterruptedException {
        Initialize(hardwareMap,mirrored);
        firstJunctionSequence = pathToFirstJunction();
        coneLineSequence = pathToConeLine();
        getConeSequence = pathToCones();
        nudgePathSequence = nudgePath();
        secondJunctionSequence = pathToSecondJunction();
        riserServo.setPosition(1);
        riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(opModeInInit()){
            //duckPos = pipeline.ComposeTelemetry(telemetry);
        }
        if(isStopRequested())return;
        currentState = State.NAVIGATING_TO_FIRST_JUNCTION;
        drive.setPoseEstimate(startPose);
        drive.followTrajectorySequenceAsync(firstJunctionSequence);
        while(opModeIsActive() && !isStopRequested()){
            switch (currentState){
                case NAVIGATING_TO_FIRST_JUNCTION:
                    // lol do something
                    if(!drive.isBusy()) {
                        currentState = State.PLACING_CONE;
                        riserServo.setPosition(0);
                        timer.reset();
                    }
                    break;
                case PLACING_CONE:
                    currentState = State.NAVIGATING_TO_CONE_STACK;
                    drive.followTrajectorySequenceAsync(coneLineSequence);
                    break;
                case NAVIGATING_TO_CONE_STACK:
                    if(!drive.isBusy()) {
                        drive.setDrivePower(new Pose2d(0,-0.15,0));
                        currentState = State.SEEKING_CONE_LINE;
                    }
                    break;
                case SEEKING_CONE_LINE:
                    if(lineSensor.blue()-lineSensor.green()>0){
                        drive.setPoseEstimate(new Pose2d(-1400,280, pi));
                        drive.followTrajectorySequenceAsync(nudgePathSequence);
                        currentState = State.ALIGNING_WITH_CONE_LINE;
                    }
                    break;
                case ALIGNING_WITH_CONE_LINE:
                    if(!drive.isBusy()){
                        drive.setPoseEstimate(new Pose2d(-1800+distanceSensor.getDistance(DistanceUnit.MM)+156,300,pi));
                        drive.followTrajectorySequenceAsync(getConeSequence);
                        currentState = State.COLLECTING_CONE;
                    }
                    break;
                case COLLECTING_CONE:
                    if(!drive.isBusy()){
                        riserMotor.setTargetPosition(armExtensionToEncoderTicks(400));
                        riserMotor.setPower(1);
                        riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        timer.reset();
                        currentState = State.PREPARING_FOR_DEPARTURE;
                    }
                    break;
                case PREPARING_FOR_DEPARTURE:
                    if(encoderTicksToArmExtension(riserMotor.getCurrentPosition())>=320){
                        drive.followTrajectorySequenceAsync(secondJunctionSequence);
                        currentState = State.NAVIGATING_TO_SECOND_JUNCTION;
                    }
                    break;
                case NAVIGATING_TO_SECOND_JUNCTION:
                    if(!drive.isBusy()){
                        currentState = State.IDLE;
                        riserServo.setPosition(0);
                        timer.reset();
                    }
                    break;
                case IDLE:
                    // lol do nothing
                    break;
                default:
                    break;
            }
            drive.update();
            telemetry.addData("State: ", currentState.name());
            telemetry.addData("Position: ", "%.3f %.3f %.3f",drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading());
            telemetry.addData("Error: ","%.3f %.3f %.3f",drive.getLastError().getX(),drive.getLastError().getY(),drive.getLastError().getHeading());
            telemetry.addData("Riser: ", "%d %d",riserMotor.getCurrentPosition(), riserMotor.getTargetPosition());
            telemetry.update();
        }
    }


    public TrajectorySequence pathToFirstJunction(){
        return drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(1400, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    riserMotor.setTargetPosition(armExtensionToEncoderTicks(890));
                    riserMotor.setPower(1);
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineToConstantHeading(new Vector2d(-400,fieldHalf-500),3*pi/2.0f)
                .splineToSplineHeading(junctionPose,Math.toRadians(-45))
                .build();
    }

    public TrajectorySequence pathToConeLine(){
        return drive.trajectorySequenceBuilder(junctionPose)
                .back(50)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    riserMotor.setTargetPosition(armExtensionToEncoderTicks(177));
                    riserMotor.setPower(0.55);
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineToSplineHeading(new Pose2d(-300,600,3*pi/2.0f),3*pi/2.0f)
                .splineToSplineHeading(new Pose2d(-600,320,pi),pi)
                .splineToConstantHeading(new Vector2d(-1400,295),pi)
                .build();
    }

    public TrajectorySequence nudgePath(){
        return drive.trajectorySequenceBuilder(new Pose2d(-1400,280, pi))
                .lineToLinearHeading(new Pose2d(-1400,300,pi))
                .build();
    }

    public TrajectorySequence pathToCones(){
        return drive.trajectorySequenceBuilder(new Pose2d(-1400,300,pi))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(600, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(-1700+160,300))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->riserServo.setPosition(1))
                .build();
    }

    public TrajectorySequence pathToSecondJunction(){
        return drive.trajectorySequenceBuilder(new Pose2d(-1700+155,300,pi))
                .back(100)
                .splineToSplineHeading(new Pose2d(-577,220,1.51*pi), 0)
                .UNSTABLE_addTemporalMarkerOffset(-0.75,()->{
                    riserMotor.setTargetPosition(armExtensionToEncoderTicks(920));
                })
                .build();
    }
}
