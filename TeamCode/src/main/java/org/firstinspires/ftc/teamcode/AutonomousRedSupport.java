package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.io.File;

@Autonomous(name = "Autonomous: RED Pos5",preselectTeleOp = "W+M1", group = "Support")
public class AutonomousRedSupport extends CommonOpMode {
    private enum AutoState {
        EN_ROUTE_TO_HUB,
        PLACING_ELEMENT,
        EN_ROUTE_TO_CAROUSEL,
        ROTATING_CAROUSEL,
        GETTING_IN_POS_TO_PICK_UP_DUCK,
        PICKING_UP_DUCK,
        EN_ROUTE_TO_HUB_WITH_DUCK,
        RETURNING_TO_DEFAULT_POS,
        RAMMING_INTO_WALL_BEFORE_ENTERING,
        PARKING,
        IDLE
    }

    AutoState currentState = AutoState.IDLE;
    ElapsedTime timer = new ElapsedTime();
    Pose2d startPoseRedSupport = startPoseRed.plus(new Pose2d(-47.125,0,0));
    Pose2d lastPose;
    @Override
    public void runOpMode() {
        Initialize(hardwareMap,true);
        riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setPoseEstimate(startPoseRedSupport);
        TrajectorySequence goToHubSequence = drive.trajectorySequenceBuilder(startPoseRedSupport)
                .strafeLeft(1)
                .splineToConstantHeading(new Vector2d(-12.5,-41),Math.toRadians(90))
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
        TrajectorySequence goToCarouselSequence = drive.trajectorySequenceBuilder(goToHubSequence.end())
                .forward(1)
                .splineToSplineHeading(new Pose2d(-fieldHalf+hWidth,-fieldHalf+hLength+4.5,Math.toRadians(90)),Math.toRadians(180))
                .build();
        TrajectorySequence pickUpDuckSequence = drive.trajectorySequenceBuilder(goToCarouselSequence.end())
                .lineTo(new Vector2d(-fieldHalf+hDiag,-fieldHalf+hDiag+4.5))
                .splineToSplineHeading(new Pose2d(-fieldHalf+hWidth+3,-fieldHalf+hLength+1.5,Math.toRadians(270 - 1e-6)),Math.toRadians(270))
                .build();
        TrajectorySequence returnWithNoDuckSequence = drive.trajectorySequenceBuilder(new Pose2d(-35,-fieldHalf+hLength+2.5,Math.toRadians(270)))
                .setReversed(true)
                .splineToSplineHeading(defaultPoseRed, Math.toRadians(270))
                .build();
        TrajectorySequence returnFromHubSequence = drive.trajectorySequenceBuilder(goToHubSequence.end())
                .forward(1)
                .splineToSplineHeading(defaultPoseRed,Math.toRadians(270))
                .build();
        TrajectorySequence parkSequence = drive.trajectorySequenceBuilder(defaultPoseRed)
                .setReversed(true)
                .lineTo(new Vector2d(fieldHalf-hLength-20,-fieldHalf+hWidth))
                .build();

        while(!opModeIsActive() && !isStopRequested()){
            duckPos = pipeline.ComposeTelemetry(telemetry);
            drive.update();
        }
        waitForStart();
        if (isStopRequested()) return;
        drive.setPoseEstimate(startPoseRedSupport);
        currentState = AutoState.EN_ROUTE_TO_HUB;
        drive.followTrajectorySequenceAsync(goToHubSequence);
        double sineStartPos = 0;
        while(opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case EN_ROUTE_TO_HUB:
                    if (!drive.isBusy()) {
                        currentState = AutoState.PLACING_ELEMENT;
                        timer.reset();
                    }
                    break;
                case PLACING_ELEMENT:
                    if (timer.time() > 0.75) {
                        freightServo.setPosition(1);
                        riserMotor.setTargetPosition(0);
                        riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        riserMotor.setPower(1);
                        duckPos = BingusPipeline.RandomizationFactor.UNDEFINED;
                        if(sineStartPos==0){
                            currentState = AutoState.EN_ROUTE_TO_CAROUSEL;
                            drive.followTrajectorySequenceAsync(goToCarouselSequence);
                        }
                        else {
                            currentState = AutoState.RETURNING_TO_DEFAULT_POS;
                            drive.followTrajectorySequenceAsync(returnFromHubSequence);
                        }
                    } else freightServo.setPosition(0);
                    break;
                case EN_ROUTE_TO_CAROUSEL:
                    if(!drive.isBusy()){
                        currentState = AutoState.ROTATING_CAROUSEL;
                        drive.setWeightedDrivePower(new Pose2d(-0.075,0.04,0));
                        timer.reset();
                    }
                    break;
                case ROTATING_CAROUSEL:
                    if(timer.time()>3){
                        drive.setWeightedDrivePower(new Pose2d());
                        currentState = AutoState.GETTING_IN_POS_TO_PICK_UP_DUCK;
                        drive.followTrajectorySequenceAsync(pickUpDuckSequence);
                        carouselMotor.setPower(0);
                    }
                    else carouselMotor.setPower(timer.time());
                    break;
                case GETTING_IN_POS_TO_PICK_UP_DUCK:
                    if(!drive.isBusy()){
                        currentState = AutoState.PICKING_UP_DUCK;
                        drive.setDrivePower(new Pose2d(0,0.1,0));
                        timer.reset();
                        sineStartPos = drive.getPoseEstimate().getX();
                        collectorMotor.setPower(1);
                    }
                    break;
                case PICKING_UP_DUCK:
                    if(hasElement() || drive.getPoseEstimate().getX()>=-35){
                        drive.setDrivePower(new Pose2d());
                        collectorMotor.setPower(0);
                        if(hasElement()){
                            currentState = AutoState.EN_ROUTE_TO_HUB_WITH_DUCK;
                            drive.runLSplineToAsync(new Pose2d(-12.5,-41.5,Math.toRadians(270)),Math.toRadians(90));
                        }
                        else {
                            currentState = AutoState.RETURNING_TO_DEFAULT_POS;
                            drive.followTrajectorySequenceAsync(returnWithNoDuckSequence);
                        }
                        timer.reset();
                    }
                    break;
                case EN_ROUTE_TO_HUB_WITH_DUCK:
                    if(!drive.isBusy()){
                        currentState = AutoState.PLACING_ELEMENT;
                        if(timer.time()<1){
                            int tgtPos = 1035;
                            if(duckPos==BingusPipeline.RandomizationFactor.LEFT) {
                                tgtPos = 250;
                            }
                            else if(duckPos == BingusPipeline.RandomizationFactor.CENTER){
                                tgtPos = 500;
                            }
                            riserMotor.setTargetPosition(tgtPos);
                            riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            riserMotor.setPower(1);
                        }
                        timer.reset();
                    }
                    if(timer.time()>=1) {
                        int tgtPos = 1035;
                        if(duckPos==BingusPipeline.RandomizationFactor.LEFT) {
                            tgtPos = 250;
                        }
                        else if(duckPos == BingusPipeline.RandomizationFactor.CENTER){
                            tgtPos = 500;
                        }
                        riserMotor.setTargetPosition(tgtPos);
                        riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        riserMotor.setPower(1);
                    }
                    break;
                case RETURNING_TO_DEFAULT_POS:
                    if(!drive.isBusy()){
                        currentState = AutoState.RAMMING_INTO_WALL_BEFORE_ENTERING;
                        drive.setWeightedDrivePower(new Pose2d(0, -0.33,0));
                        timer.reset();
                    }
                    break;
                case RAMMING_INTO_WALL_BEFORE_ENTERING:
                    if(((StandardTrackingWheelLocalizer) drive.getLocalizer()).getWheelVelocities().get(2)<0.3 && timer.time()>0.5){
                        drive.setWeightedDrivePower(new Pose2d(0,0,0));
                        drive.setPoseEstimate(defaultPoseRed);
                        currentState = AutoState.PARKING;
                        drive.followTrajectorySequenceAsync(parkSequence);
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
            telemetry.addData("Dead wheel velo: ","%.3f %.3f %.3f", ((StandardTrackingWheelLocalizer) drive.getLocalizer()).getWheelVelocities().toArray());
            telemetry.addData("Riser: ", riserMotor.getCurrentPosition());
            telemetry.update();
        }
        String filename = "LastPosition";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, lastPose.getX()+" "+lastPose.getY()+" "+lastPose.getHeading());
    }
}

