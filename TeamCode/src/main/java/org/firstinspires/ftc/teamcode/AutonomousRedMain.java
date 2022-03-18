package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(preselectTeleOp = "W+M1", name = "Autonomous: RED", group = "Main")
public class AutonomousRedMain extends CommonOpMode {
    private enum AutoState {
        EN_ROUTE_TO_HUB,
        PLACING_ELEMENT,
        RETURNING_TO_DEFAULT_POS_FROM_HUB,
        RAMMING_INTO_WALL_BEFORE_ENTERING,
        RAMMING_INTO_WALL_BEFORE_LEAVING,
        EN_ROUTE_TO_WAREHOUSE,
        GETTING_ELEMENT,
        RESETTING_POSITION_IN_WAREHOUSE,
        RETURNING_TO_DEFAULT_POS_FROM_WAREHOUSE,
        PARKING,
        IDLE
    }

    AutoState currentState = AutoState.IDLE, prevState = AutoState.IDLE;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        Initialize(hardwareMap,true);
        riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setPoseEstimate(startPoseRed);
        TrajectorySequence goToHubSequence = drive.trajectorySequenceBuilder(startPoseRed)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-12.5,-41),Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{
                    int tgtPos = 1035;
                    if(duckPos==BingusPipeline.RandomizationFactor.LEFT) {
                        tgtPos = 100;
                    }
                    else if(duckPos == BingusPipeline.RandomizationFactor.CENTER){
                        tgtPos = 500;
                    }
                    riserMotor.setTargetPosition(tgtPos);
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    riserMotor.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.05,()->freightServo.setPosition(0))
                .build();
        TrajectorySequence returnFromHubSequence = drive.trajectorySequenceBuilder(goToHubSequence.end())
                .forward(1)
                .splineToSplineHeading(defaultPoseRed,Math.toRadians(270))
                .build();
        TrajectorySequence enterWarehouseSequence = drive.trajectorySequenceBuilder(defaultPoseRed)
                .setReversed(true)
                .lineTo(warehousePoseRed.vec())
                .build();
        TrajectorySequence exitWarehouseSequence = drive.trajectorySequenceBuilder(warehousePoseRed)
                .setReversed(true)
                .lineTo(new Vector2d(fieldHalf-hLength-50,-fieldHalf+hWidth))
                .splineToLinearHeading(startPoseRed,Math.toRadians(270))
                .build();


        int amountOfDeliveredElements = 0;
        final int targetDeliveries = 2;
        while(!opModeIsActive() && !isStopRequested()){
            duckPos = pipeline.ComposeTelemetry(telemetry);
        }
        waitForStart();
        if (isStopRequested()) return;
        riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        currentState = AutoState.EN_ROUTE_TO_HUB;
        prevState = AutoState.EN_ROUTE_TO_HUB;
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
                    if (timer.time()>0.35){
                        duckPos = BingusPipeline.RandomizationFactor.UNDEFINED;
                        currentState = AutoState.RETURNING_TO_DEFAULT_POS_FROM_HUB;
                        freightServo.setPosition(1);
                        riserMotor.setTargetPosition(0);
                        riserMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        amountOfDeliveredElements++;
                        drive.followTrajectorySequenceAsync(returnFromHubSequence);
                    }
                    else freightServo.setPosition(0);
                    break;
                case RETURNING_TO_DEFAULT_POS_FROM_HUB:
                    if(!drive.isBusy()){
                        currentState = AutoState.RAMMING_INTO_WALL_BEFORE_ENTERING;
                        drive.setWeightedDrivePower(new Pose2d(0, -0.25,0));
                        riserMotor.setPower(0);
                        timer.reset();
                    }
                    break;
                case RAMMING_INTO_WALL_BEFORE_ENTERING:
                    if(((StandardTrackingWheelLocalizer) drive.getLocalizer()).getWheelVelocities().get(2)>-0.2 && timer.time()>0.2){
                        drive.setWeightedDrivePower(new Pose2d(0,0,0));
                        drive.setPoseEstimate(defaultPoseRed);
                        currentState = AutoState.EN_ROUTE_TO_WAREHOUSE;
                        drive.followTrajectorySequenceAsync(enterWarehouseSequence);
                    }
                    break;
                case EN_ROUTE_TO_WAREHOUSE:
                    if(!drive.isBusy()){
                        currentState = AutoState.GETTING_ELEMENT;
                        timer.reset();
                        drive.setWeightedDrivePower(new Pose2d(0.1,0,0));
                        collectorMotor.setPower(1);
                    }
                    break;
                case GETTING_ELEMENT:
                    if(hasElement() || drive.getPoseEstimate().getX()>=fieldHalf-hLength){
                        freightServo.setPosition(0.7);
                        drive.setWeightedDrivePower(new Pose2d(0,0,0));
                        collectorMotor.setPower(-1);
                        if(amountOfDeliveredElements<targetDeliveries){
                            currentState = AutoState.RESETTING_POSITION_IN_WAREHOUSE;
                            drive.runConstantSplineToAsync(warehousePoseRed,Math.toRadians(270),true);
                        }
                        else{
                            currentState = AutoState.PARKING;
                            drive.runConstantSplineToAsync(warehousePoseRed.plus(new Pose2d(0,20,0)),Math.toRadians(90),true);
                            riserMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        }
                    }
                    break;
                case RESETTING_POSITION_IN_WAREHOUSE:
                    if(!drive.isBusy()){
                        drive.setPoseEstimate(warehousePoseRed);
                        collectorMotor.setPower(0);
                        currentState = AutoState.RAMMING_INTO_WALL_BEFORE_LEAVING;
                        drive.setWeightedDrivePower(new Pose2d(0, -0.25,0));
                        timer.reset();
                    }
                    break;
                case RAMMING_INTO_WALL_BEFORE_LEAVING:
                    if(((StandardTrackingWheelLocalizer) drive.getLocalizer()).getWheelVelocities().get(2)>-0.2 && timer.time()>0.2){
                        drive.setWeightedDrivePower(new Pose2d(0,0,0));
                        drive.setPoseEstimate(warehousePoseRed);
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
                        collectorMotor.setPower(0);
                        currentState = AutoState.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }
            drive.update();

            if(riserMotor.getMode()== DcMotor.RunMode.RUN_WITHOUT_ENCODER){
                riserMotor.setPower(armButton.isPressed()?0:-0.5);
            }
            telemetry.addData("State: ", currentState.name());
            telemetry.addData("Position: ", "%.3f %.3f %.3f",drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading());
            telemetry.addData("Error: ","%.3f %.3f %.3f",drive.getLastError().getX(),drive.getLastError().getY(),drive.getLastError().getHeading());
            telemetry.addData("Amount of delivered freight: ", amountOfDeliveredElements);
            telemetry.addData("Dead wheel velo: ","%.3f %.3f %.3f", ((StandardTrackingWheelLocalizer) drive.getLocalizer()).getWheelVelocities().toArray());
            telemetry.addData("Riser: ", riserMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
