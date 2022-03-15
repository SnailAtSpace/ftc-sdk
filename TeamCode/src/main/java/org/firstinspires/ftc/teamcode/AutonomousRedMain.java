package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.io.File;

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
    Pose2d lastPose;
    @Override
    public void runOpMode() throws InterruptedException {
        Initialize(hardwareMap,true);
        riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setPoseEstimate(startPoseRed);
        TrajectorySequence goToHubSequence = drive.trajectorySequenceBuilder(startPoseRed)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-12.5,-41.5),Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{
                    int tgtPos = 1035;
                    if(duckPos==BingusPipeline.RandomizationFactor.LEFT) {
                        tgtPos = 50;
                    }
                    else if(duckPos == BingusPipeline.RandomizationFactor.CENTER){
                        tgtPos = 500;
                    }
                    riserMotor.setTargetPosition(tgtPos);
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    riserMotor.setPower(1);
                })
                .build();
        TrajectorySequence returnFromHubSequence = drive.trajectorySequenceBuilder(goToHubSequence.end())
                .forward(1)
                .splineToSplineHeading(defaultPoseRed,Math.toRadians(270))
                .build();
        TrajectorySequence enterWarehouseSequence = drive.trajectorySequenceBuilder(defaultPoseRed)
                .setReversed(true)
                .lineTo(warehousePoseRed.vec())
                .splineToConstantHeading(warehousePoseRed.vec().plus(new Vector2d(0,5)),Math.toRadians(90))
                .build();
        TrajectorySequence exitWarehouseSequence = drive.trajectorySequenceBuilder(warehousePoseRed)
                .setReversed(true)
                .lineTo(new Vector2d(fieldHalf-hLength-50,-fieldHalf+hWidth))
                .splineToLinearHeading(startPoseRed,Math.toRadians(270))
                .build();
        TrajectorySequence parkSequence = drive.trajectorySequenceBuilder(defaultPoseRed)
                .lineTo(warehousePoseRed.vec())
                .splineToConstantHeading(new Vector2d(fieldHalf-hLength-20,-fieldHalf+hWidth+20),Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .build();


        int amountOfDeliveredElements = 0;
        final int targetDeliveries = 2;
        while(!opModeIsActive() && !isStopRequested()){
            duckPos = pipeline.ComposeTelemetry(telemetry);
        }
        waitForStart();
        if (isStopRequested()) return;
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
                    if (timer.time()>0.75){
                        duckPos = BingusPipeline.RandomizationFactor.UNDEFINED;
                        currentState = AutoState.RETURNING_TO_DEFAULT_POS_FROM_HUB;
                        freightServo.setPosition(1);
                        riserMotor.setTargetPosition(0);
                        riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        riserMotor.setPower(1);
                        amountOfDeliveredElements++;
                        drive.followTrajectorySequenceAsync(returnFromHubSequence);
                    }
                    else freightServo.setPosition(0);
                    break;
                case RETURNING_TO_DEFAULT_POS_FROM_HUB:
                    if(!drive.isBusy()){
                        currentState = AutoState.RAMMING_INTO_WALL_BEFORE_ENTERING;
                        drive.setWeightedDrivePower(new Pose2d(0, -0.33,0));
                        timer.reset();
                    }
                    break;
                case RAMMING_INTO_WALL_BEFORE_ENTERING:
                    if(((StandardTrackingWheelLocalizer) drive.getLocalizer()).getWheelVelocities().get(2)<0.3 && timer.time()>0.25){
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
                    break;
                case EN_ROUTE_TO_WAREHOUSE:
                    if(!drive.isBusy()){
                        currentState = AutoState.GETTING_ELEMENT;
                        timer.reset();
                        drive.setWeightedDrivePower(new Pose2d(0.15,0,0));
                        collectorMotor.setPower(1);
                    }
                    break;
                case GETTING_ELEMENT:
                    if(hasElement() || drive.getPoseEstimate().getX()>=fieldHalf-hLength || timer.time()>3){
                        freightServo.setPosition(0.9);
                        drive.setWeightedDrivePower(new Pose2d(0,0,0));
                        collectorMotor.setPower(-1);
                        currentState = AutoState.RESETTING_POSITION_IN_WAREHOUSE;
                        drive.runConstantSplineToAsync(warehousePoseRed.plus(new Pose2d(0,5,0)),Math.toRadians(270));
                    }
                    break;
                case RESETTING_POSITION_IN_WAREHOUSE:
                    if(!drive.isBusy()){
                        drive.setPoseEstimate(warehousePoseRed.plus(new Pose2d(0,5,0)));
                        collectorMotor.setPower(0);
                        currentState = AutoState.RAMMING_INTO_WALL_BEFORE_LEAVING;
                        drive.setWeightedDrivePower(new Pose2d(0, -0.33,0));
                        timer.reset();
                    }
                    break;
                case RAMMING_INTO_WALL_BEFORE_LEAVING:
                    if(((StandardTrackingWheelLocalizer) drive.getLocalizer()).getWheelVelocities().get(2)<0.3 && timer.time()>0.25){
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
                        currentState = AutoState.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }
            drive.update();
            lastPose = drive.getPoseEstimate();
            telemetry.addData("State: ", currentState.name());
            telemetry.addData("Position: ", "%.3f %.3f %.3f",drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading());
            telemetry.addData("Error: ","%.3f %.3f %.3f",drive.getLastError().getX(),drive.getLastError().getY(),drive.getLastError().getHeading());
            telemetry.addData("Amount of delivered freight: ", amountOfDeliveredElements);
            telemetry.addData("Dead wheel velo: ","%.3f %.3f %.3f", ((StandardTrackingWheelLocalizer) drive.getLocalizer()).getWheelVelocities().toArray());
            telemetry.addData("Riser: ", riserMotor.getCurrentPosition());
            telemetry.update();
        }
        String filename = "LastPosition";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, lastPose.getX()+" "+lastPose.getY()+" "+lastPose.getHeading());
    }
}
