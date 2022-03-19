package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Autonomous: BLUE Pos5",preselectTeleOp = "W+M1", group = "Support")
public class AutonomousBlueSupport extends CommonOpMode {
    private enum AutoState {
        EN_ROUTE_TO_HUB,
        PLACING_ELEMENT,
        EN_ROUTE_TO_CAROUSEL,
        ROTATING_CAROUSEL,
        GETTING_IN_POS_TO_PICK_UP_DUCK,
        PICKING_UP_DUCK,
        RETURNING_TO_DEFAULT_POS,
        RAMMING_INTO_WALL_BEFORE_ENTERING,
        EN_ROUTE_TO_WAREHOUSE,
        GETTING_ELEMENT,
        PARKING,
        IDLE
    }

    AutoState currentState = AutoState.IDLE;
    ElapsedTime timer = new ElapsedTime();
    Pose2d startPoseBlueSupport = startPoseBlue.plus(new Pose2d(-47.125,0,0));
    @Override
    public void runOpMode() {
        Initialize(hardwareMap,true);
        riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setPoseEstimate(startPoseBlueSupport);
        TrajectorySequence goToHubSequence = drive.trajectorySequenceBuilder(startPoseBlueSupport)
                .strafeRight(1)
                .splineToConstantHeading(new Vector2d(-12.5,40.5),Math.toRadians(270))
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
        TrajectorySequence goToCarouselSequence = drive.trajectorySequenceBuilder(goToHubSequence.end())
                .forward(1)
                .splineToSplineHeading(new Pose2d(-fieldHalf+hLength,fieldHalf-hWidth-4.25,Math.toRadians(0)),Math.toRadians(90))
                .build();
        TrajectorySequence pickUpDuckSequence = drive.trajectorySequenceBuilder(goToCarouselSequence.end())
                .lineTo(new Vector2d(-fieldHalf+hDiag,fieldHalf-hDiag-4.5))
                .splineToSplineHeading(new Pose2d(-fieldHalf+hWidth+3,fieldHalf-hLength-3,Math.toRadians(90)),Math.toRadians(90))
                .build();
        TrajectorySequence returnWithNoDuckSequence = drive.trajectorySequenceBuilder(new Pose2d(-35,fieldHalf-hLength-1.5,Math.toRadians(90)))
                .setReversed(true)
                .splineToSplineHeading(defaultPoseBlue, Math.toRadians(90))
                .build();
        TrajectorySequence returnFromHubSequence = drive.trajectorySequenceBuilder(goToHubSequence.end())
                .forward(1)
                .splineToSplineHeading(defaultPoseBlue,Math.toRadians(90))
                .build();
        TrajectorySequence enterWarehouseSequence = drive.trajectorySequenceBuilder(defaultPoseBlue)
                .setReversed(true)
                .lineTo(warehousePoseBlue.vec())
                .build();
        TrajectorySequence parkSequence = drive.trajectorySequenceBuilder(new Pose2d(fieldHalf-23.5,fieldHalf-hWidth,Math.toRadians(0)))
                .lineTo(warehousePoseBlue.plus(new Pose2d(0,0,0)).vec())
                .build();

        while(!opModeIsActive() && !isStopRequested()){
            duckPos = pipeline.ComposeTelemetry(telemetry);
            drive.update();
        }
        waitForStart();
        if (isStopRequested()) return;
        riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setPoseEstimate(startPoseBlueSupport);
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
                    if (timer.time() > 0.35) {
                        freightServo.setPosition(1);
                        riserMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
                        drive.setWeightedDrivePower(new Pose2d(-0.04,0.075,0));
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
                    else carouselMotor.setPower(-timer.time());
                    break;
                case GETTING_IN_POS_TO_PICK_UP_DUCK:
                    if(!drive.isBusy()){
                        currentState = AutoState.PICKING_UP_DUCK;
                        drive.setDrivePower(new Pose2d(0,-0.15,0));
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
                            currentState = AutoState.EN_ROUTE_TO_HUB;
                            drive.followTrajectorySequenceAsync(goToHubSequence);
                            timer.reset();
                        }
                        else {
                            currentState = AutoState.RETURNING_TO_DEFAULT_POS;
                            drive.followTrajectorySequenceAsync(returnWithNoDuckSequence);
                        }
                        timer.reset();
                    }
                    break;
                case RETURNING_TO_DEFAULT_POS:
                    if(!drive.isBusy()){
                        currentState = AutoState.RAMMING_INTO_WALL_BEFORE_ENTERING;
                        drive.setWeightedDrivePower(new Pose2d(0, 0.25,0));
                        timer.reset();
                    }
                    break;
                case RAMMING_INTO_WALL_BEFORE_ENTERING:
                    if(((StandardTrackingWheelLocalizer) drive.getLocalizer()).getWheelVelocities().get(2)<0.2 && timer.time()>0.2){
                        drive.setWeightedDrivePower(new Pose2d(0,0,0));
                        drive.setPoseEstimate(defaultPoseBlue);
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
                        riserMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        currentState = AutoState.PARKING;
                        drive.followTrajectorySequenceAsync(parkSequence);
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
                riserMotor.setPower(armButton.isPressed()?0:Math.max(-0.45,-0.001*(riserMotor.getCurrentPosition())-0.1));
            }
            telemetry.addData("State: ", currentState.name());
            telemetry.addData("Position: ", "%.3f %.3f %.3f",drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading());
            telemetry.addData("Error: ","%.3f %.3f %.3f",drive.getLastError().getX(),drive.getLastError().getY(),drive.getLastError().getHeading());
            telemetry.addData("Dead wheel velo: ","%.3f %.3f %.3f", ((StandardTrackingWheelLocalizer) drive.getLocalizer()).getWheelVelocities().toArray());
            telemetry.addData("Riser: ", riserMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}

