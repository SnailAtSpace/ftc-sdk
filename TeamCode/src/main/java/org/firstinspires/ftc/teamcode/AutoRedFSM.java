package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Locale;

@Autonomous
public class AutoRedFSM extends CommonOpMode {
    enum AutoState {
        EN_ROUTE_TO_HUB,
        PLACING_ELEMENT,
        RETURNING_TO_DEFAULT_POS_FROM_HUB,
        RAMMING_INTO_WALL,
        EN_ROUTE_TO_WAREHOUSE,
        GETTING_ELEMENT,
        RESETTING_POSITION_IN_WAREHOUSE,
        RETURNING_TO_DEFAULT_POS_FROM_WAREHOUSE,
        PARKING,
        IDLE
    }

    TrajectorySequence goToHubSequence = drive.trajectorySequenceBuilder(startPoseRed)
            .setReversed(true)
            .splineToConstantHeading(new Vector2d(-12.5,-41.5),Math.toRadians(90))
            .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                int tgtPos = 600;
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
            .build();
    TrajectorySequence returnFromHubSequence = drive.trajectorySequenceBuilder(goToHubSequence.end())
            .setReversed(true)
            .forward(0.5)
            .splineToLinearHeading(defaultPoseRed,Math.toRadians(270))
            .build();
    TrajectorySequence enterWarehouseSequence = drive.trajectorySequenceBuilder(defaultPoseRed)
            .setReversed(true)
            .lineTo(new Vector2d(fieldHalf-hLength-27,-fieldHalf+hWidth))
            .splineToConstantHeading(new Vector2d(fieldHalf-hLength-27,-fieldHalf+hWidth+2),Math.toRadians(90))
            .build();
    TrajectorySequence exitWarehouseSequence = drive.trajectorySequenceBuilder(enterWarehouseSequence.end())
            .setReversed(true)
            .lineTo(defaultPoseRed.vec())
            .splineToLinearHeading(startPoseRed,Math.toRadians(270))
            .build();
    TrajectorySequence parkSequence = drive.trajectorySequenceBuilder(defaultPoseRed)
            .setReversed(true)
            .lineTo(new Vector2d(fieldHalf-hLength-20,-fieldHalf+hWidth))
            .splineToConstantHeading(new Vector2d(fieldHalf-hLength-20,-fieldHalf+hDiag+0.5),90)
            .lineTo(new Vector2d(fieldHalf-20-hLength,-fieldHalf+hWidth+27.5))
            .build();
    AutoState currentState = AutoState.IDLE;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        int amountOfDeliveredElements = 0;
        final int targetDeliveries = 2;
        Initialize(hardwareMap,true);
        drive.setPoseEstimate(startPoseRed);
        waitForStart();
        if (isStopRequested()) return;
        currentState = AutoState.EN_ROUTE_TO_HUB;
        drive.followTrajectorySequenceAsync(goToHubSequence);
        while(opModeIsActive() && !isStopRequested()){
            switch (currentState){
                case EN_ROUTE_TO_HUB:
                    if(!drive.isBusy()){
                        currentState = AutoState.PLACING_ELEMENT;
                        timer.reset();
                    }
                    break;
                case PLACING_ELEMENT:
                    if(timer.time()<0.25){
                        freightServo.setPosition(0);
                    }
                    else if (timer.time()<1){
                        freightServo.setPosition(1);
                        riserMotor.setTargetPosition(0);
                        riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        riserMotor.setPower(1);
                    }
                    if(riserMotor.getCurrentPosition()<150){
                        amountOfDeliveredElements++;
                        duckPos = BingusPipeline.RandomizationFactor.UNDEFINED;
                        currentState = AutoState.RETURNING_TO_DEFAULT_POS_FROM_HUB;
                        drive.followTrajectorySequenceAsync(returnFromHubSequence);
                    }
                    break;
                case RETURNING_TO_DEFAULT_POS_FROM_HUB:
                    if(!drive.isBusy()){
                        currentState = AutoState.RAMMING_INTO_WALL;
                        drive.setWeightedDrivePower(new Pose2d(0, -0.25,0));
                    }
                    break;
                case RAMMING_INTO_WALL:
                    if(((StandardTrackingWheelLocalizer) drive.getLocalizer()).getWheelVelocities().get(2)<0.5){
                        drive.setWeightedDrivePower(new Pose2d(0,0,0));
                        drive.setPoseEstimate(defaultPoseRed);
                        if(amountOfDeliveredElements<targetDeliveries){ // get more elements
                            currentState = AutoState.EN_ROUTE_TO_WAREHOUSE;
                            drive.followTrajectorySequenceAsync(enterWarehouseSequence);
                        }
                        else{
                            currentState = AutoState.PARKING;
                            drive.followTrajectorySequenceAsync(parkSequence);
                        }
                    }
                    else {
                        drive.update();
                    }
                    break;
                case EN_ROUTE_TO_WAREHOUSE:
                    if(!drive.isBusy()){
                        currentState = AutoState.GETTING_ELEMENT;
                        drive.setWeightedDrivePower(new Pose2d(0.2,0,0));
                        collectorMotor.setPower(1);
                    }
                    break;
                case GETTING_ELEMENT:
                    if(freightSensor.hasElement() || drive.getPoseEstimate().getX()>=fieldHalf-hLength){
                        drive.setWeightedDrivePower(new Pose2d(0,0,0));
                        collectorMotor.setPower(-1);
                        currentState = AutoState.RESETTING_POSITION_IN_WAREHOUSE;
                        drive.runSplineToAsync(enterWarehouseSequence.end(),Math.toRadians(180));
                    }
                    break;
                case RESETTING_POSITION_IN_WAREHOUSE:
                    if(!drive.isBusy()){
                        drive.setPoseEstimate(enterWarehouseSequence.end());
                        collectorMotor.setPower(0);
                        currentState = AutoState.RETURNING_TO_DEFAULT_POS_FROM_WAREHOUSE;
                        drive.followTrajectorySequenceAsync(exitWarehouseSequence);
                    }
                    break;
                case RETURNING_TO_DEFAULT_POS_FROM_WAREHOUSE:
                    if(!drive.isBusy()){
                        drive.setPoseEstimate(startPoseRed);
                        currentState = AutoState.EN_ROUTE_TO_HUB;
                        drive.followTrajectorySequenceAsync(goToHubSequence);
                    }
                    break;
                case PARKING:
                    if(!drive.isBusy()){
                        currentState = AutoState.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }
            telemetry.addData("State: ", currentState.name());
            telemetry.addData("Position: ", String.format(Locale.ENGLISH,"%.3f %.3f %.3f"),drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading());
            telemetry.addData("Error: ",String.format(Locale.ENGLISH,"%.3f %.3f %.3f",drive.getLastError().getX(),drive.getLastError().getY(),drive.getLastError().getHeading()));
            telemetry.addData("Amount of delivered freight: ", amountOfDeliveredElements);
        }
    }
}
