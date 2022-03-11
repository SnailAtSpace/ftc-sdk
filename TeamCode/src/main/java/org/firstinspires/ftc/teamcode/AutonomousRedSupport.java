package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Autonomous: RED Pos5",preselectTeleOp = "W+M1")
public class AutonomousRedSupport extends CommonOpMode {
    enum AutoState {
        EN_ROUTE_TO_HUB,
        PLACING_ELEMENT,
        EN_ROUTE_TO_CAROUSEL,
        ROTATING_CAROUSEL,
        GETTING_IN_POS_TO_PICK_UP_DUCK,
        PICKING_UP_DUCK,
        EN_ROUTE_TO_HUB_WITH_DUCK,
        RETURNING_TO_DEFAULT_POS_FROM_HUB,
        RAMMING_INTO_WALL_BEFORE_ENTERING,
        PARKING,
        IDLE
    }

    AutoState currentState = AutoState.IDLE;
    ElapsedTime timer = new ElapsedTime();
    Pose2d startPoseRedSupport = startPoseRed.plus(new Pose2d(-47.125,0,0));
    @Override
    public void runOpMode() {
        Initialize(hardwareMap,true);
        riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setPoseEstimate(startPoseRedSupport);
        TrajectorySequence goToHubSequence = drive.trajectorySequenceBuilder(startPoseRedSupport)
                .setReversed(true)
                .strafeLeft(1)
                .splineToConstantHeading(new Vector2d(-12.5,-41.5),Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{
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
                })
                .build();
        TrajectorySequence goToCarouselSequence = drive.trajectorySequenceBuilder(goToHubSequence.end())
                .setReversed(true)
                .forward(1)
                .splineToSplineHeading(new Pose2d(-fieldHalf+hWidth,-fieldHalf+hLength+5,Math.toRadians(90)),Math.toRadians(180))
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
                        if (riserMotor.getCurrentPosition() < 400) {
                            duckPos = BingusPipeline.RandomizationFactor.UNDEFINED;
                            currentState = AutoState.EN_ROUTE_TO_CAROUSEL;
                            drive.followTrajectorySequenceAsync(goToCarouselSequence);

                        } else {
                            freightServo.setPosition(1);
                            riserMotor.setTargetPosition(0);
                            riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            riserMotor.setPower(1);
                        }
                    } else freightServo.setPosition(0);
                    break;
                case EN_ROUTE_TO_CAROUSEL:
                    if(!drive.isBusy()){
                        currentState = AutoState.ROTATING_CAROUSEL;
                        timer.reset();
                        carouselMotor.setPower(1);
                    }
                case ROTATING_CAROUSEL:
                    if(timer.time()>3){
                        currentState = AutoState.IDLE;
                        carouselMotor.setPower(0);
                    }
                    break;
                case IDLE:
                    break;
            }
            drive.update();
            telemetry.addData("State: ", currentState.name());
            telemetry.addData("Position: ", "%.3f %.3f %.3f",drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading());
            telemetry.addData("Error: ","%.3f %.3f %.3f",drive.getLastError().getX(),drive.getLastError().getY(),drive.getLastError().getHeading());
            telemetry.addData("Dead wheel velo: ","%.3f %.3f %.3f", ((StandardTrackingWheelLocalizer) drive.getLocalizer()).getWheelVelocities().toArray());
            telemetry.addData("Riser: ", riserMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}

