package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public abstract class AutonomousBoilerplate extends AutoOpMode{
    protected Pose2d startPose = new Pose2d(-590,fieldHalf-hWidth,Math.toRadians(0.6));
    protected TrajectorySequence firstJunctionSequence,getConeSequence,nudgePathSequence,coneLineSequence,secondJunctionSequence,nextConeSequence,parkingSequence;

    public void runOpMode(boolean mirrored) throws InterruptedException {
        int conesCollected = 0, coneStackHeight = 165;
        Initialize(hardwareMap,mirrored);
        drive.setCorrectedPoseEstimate(startPose);
        drive.update();
        firstJunctionSequence = pathToFirstJunction();
        coneLineSequence = pathToConeLine();
        getConeSequence = pathToCones();
        nudgePathSequence = nudgePath();
        secondJunctionSequence = pathToSecondJunction();
        nextConeSequence = pathToConesFromSecondJunction();
        parkingSequence = pathToParking();
        riserServo.setPosition(1);
        riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addLine("Initialization complete; robot ready for match start.");
        while(opModeInInit()){
            //duckPos = pipeline.ComposeTelemetry(telemetry);
        }
        if(isStopRequested())return;
        currentState = State.NAVIGATING_TO_FIRST_JUNCTION;
        drive.setCorrectedPoseEstimate(startPose);
        drive.followTrajectorySequenceAsync(firstJunctionSequence);
        while(opModeIsActive() && !isStopRequested()){
            switch (currentState){
                case NAVIGATING_TO_FIRST_JUNCTION:
                    // lol do something
                    if(!drive.isBusy()) {
                        currentState = State.NAVIGATING_TO_CONE_STACK;
                        riserServo.setPosition(0);
                        drive.followTrajectorySequenceAsync(coneLineSequence);
                        timer.reset();
                    }
                    break;
                case NAVIGATING_TO_CONE_STACK:
                    if(!drive.isBusy()) {
                        drive.setDrivePower(new Pose2d(0,-0.15,0));
                        currentState = State.SEEKING_CONE_LINE;
                    }
                    else if(timer.seconds()>=0.2){
                        riserMotor.setTargetPosition(armExtensionToEncoderTicks(coneStackHeight));
                        riserMotor.setPower(0.65);
                    }
                    break;
                case SEEKING_CONE_LINE:
                    if(lineSensor.blue()-lineSensor.green()>0){
                        drive.setDrivePower(new Pose2d());
                        drive.setCorrectedPoseEstimate(new Pose2d(-1800+distanceSensor.getDistance(DistanceUnit.MM)+150,285, pi));
                        drive.followTrajectorySequenceAsync(getConeSequence);
                        currentState = State.COLLECTING_CONE;
                    }
                    break;
                case COLLECTING_CONE:
                    if(!drive.isBusy()){
                        coneStackHeight-=35;
                        riserMotor.setTargetPosition(armExtensionToEncoderTicks(400));
                        riserMotor.setPower(1);
                        timer.reset();
                        currentState = State.PREPARING_FOR_DEPARTURE;
                    }
                    break;
                case PREPARING_FOR_DEPARTURE:
                    if(encoderTicksToArmExtension(riserMotor.getCurrentPosition())>=coneStackHeight+155){
                        drive.followTrajectorySequenceAsync(secondJunctionSequence);
                        currentState = State.NAVIGATING_TO_SECOND_JUNCTION;
                    }
                    break;
                case NAVIGATING_TO_SECOND_JUNCTION:
                    if(!drive.isBusy()){
                        riserServo.setPosition(0);
                        drive.setPoseEstimate(drive.getPoseEstimate().minus(new Pose2d(0,0,Math.toRadians(2))));
                        conesCollected++;
                        if(conesCollected==maxCones){
                            currentState = State.IDLE;
                            //drive.followTrajectorySequenceAsync(parkingSequence);
                        }
                        else{
                            currentState = State.NAVIGATING_TO_CONE_STACK;
                            drive.followTrajectorySequenceAsync(nextConeSequence);
                        }
                        timer.reset();
                    }
                    break;
                case PARKING:
                    if(!drive.isBusy()){
                        currentState = State.IDLE;
                    }
                case IDLE:
                    // lol do nothing
                    break;
                default:
                    break;
            }
            drive.update();
            telemetry.addData("Error: ","%.3f %.3f %.3f",drive.getLastError().getX(),drive.getLastError().getY(),drive.getLastError().getHeading()*180/pi);
            telemetry.update();
        }
    }


    public TrajectorySequence pathToFirstJunction(){
        return drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(1400, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    riserMotor.setTargetPosition(armExtensionToEncoderTicks(920));
                    riserMotor.setPower(1);
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .lineTo(startPose.plus(new Pose2d(5,-5,0)).vec())
                .splineToConstantHeading(new Vector2d(-400,fieldHalf-500),3*pi/2.0f)
                .splineToSplineHeading(junctionPose,Math.toRadians(-45))
                .build();
    }

    public TrajectorySequence pathToConeLine(){
        return drive.trajectorySequenceBuilder(junctionPose)
                .back(70)
                .splineToSplineHeading(new Pose2d(-320,600,3*pi/2.0f),3*pi/2.0f)
                .splineToSplineHeading(new Pose2d(-600,310,pi),pi)
                .splineToConstantHeading(new Vector2d(-1400,275),1.1*pi)
                .build();
    }

    public TrajectorySequence pathToConesFromSecondJunction(){
        return drive.trajectorySequenceBuilder(secondJunctionPose)
                .lineTo(secondJunctionPose.plus(new Pose2d(-5,5,0)).vec())
                .splineToSplineHeading(new Pose2d(-1400,275,pi),1.1*pi)
                .build();
    }

    public TrajectorySequence nudgePath(){
        return drive.trajectorySequenceBuilder(new Pose2d(-1400,285, pi))
                .lineToLinearHeading(new Pose2d(-1400,300,pi))
                .build();
    }

    public TrajectorySequence pathToCones(){
        return drive.trajectorySequenceBuilder(new Pose2d(-1400,285,pi))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(700, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(-1700+165,300), pi)
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->riserServo.setPosition(1))
                .build();
    }

    public TrajectorySequence pathToSecondJunction(){
        return drive.trajectorySequenceBuilder(conePose)
                .lineTo(new Vector2d(-1200,350))
                .splineToSplineHeading(secondJunctionPose, -0.1*pi)
                .UNSTABLE_addTemporalMarkerOffset(-1.175,()->{
                    riserMotor.setTargetPosition(armExtensionToEncoderTicks(940));
                    riserMotor.setPower(1);
                })
                .build();
    }

    public TrajectorySequence pathToParking(){
        return drive.trajectorySequenceBuilder(secondJunctionPose)
                .back(10)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    riserMotor.setTargetPosition(0);
                    riserMotor.setPower(0.65);
                })
                .splineToConstantHeading(new Vector2d(-950,600),0.5*pi)
                .splineToConstantHeading(new Vector2d(-900,1200), 0.5*pi)
                .splineToConstantHeading(new Vector2d(-1500,1500), pi)
                .build();
    }
}
