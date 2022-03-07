package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class AutoRedFSM extends CommonOpMode {
    enum AutoState {
        EN_ROUTE_TO_HUB,
        PLACING_ELEMENT,
        RETURNING_TO_DEFAULT_POS_FROM_HUB,
        EN_ROUTE_TO_WAREHOUSE,
        GETTING_ELEMENT,
        RETURNING_TO_DEFAULT_POS_FROM_WAREHOUSE,
        PARKING,
        IDLE
    }

    AutoState currentState = AutoState.IDLE;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Initialize(hardwareMap,true);
        drive.setPoseEstimate(startPoseRed);
        TrajectorySequence goToHubSequence = drive.trajectorySequenceBuilder(startPoseRed)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-12.5,-41.5),Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    int tgtPos = 600;
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(duckPos== BingusPipeline.RandomizationFactor.LEFT) {
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
        TrajectorySequence firstFreightSequence = drive.trajectorySequenceBuilder(defaultPoseRed)
                .setReversed(true)
                .lineTo(new Vector2d(fieldHalf-hLength-20,-fieldHalf+hWidth))
                .UNSTABLE_addTemporalMarkerOffset(-1,()-> collectorMotor.setPower(maxCollPower*0.85))
                .build();
        TrajectorySequence enterWarehouseSequence = drive.trajectorySequenceBuilder(defaultPoseRed)
                .setReversed(true)
                .lineTo(new Vector2d(fieldHalf-hLength-27,-fieldHalf+hWidth))
                .UNSTABLE_addTemporalMarkerOffset(1,()-> collectorMotor.setPower(maxCollPower*0.85))
                .splineToConstantHeading(new Vector2d(fieldHalf-hLength-27,-fieldHalf+hWidth+2),Math.toRadians(0))
                .build();
        TrajectorySequence exitWarehouseSequence = drive.trajectorySequenceBuilder(enterWarehouseSequence.end())
                .setReversed(true)
                .lineTo(defaultPoseRed.vec())
                .build();
        TrajectorySequence deliverSequence = drive.trajectorySequenceBuilder(defaultPoseRed)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(7.5,-fieldHalf+hDiag),Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-12.5,-41.5, Math.toRadians(270)),Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    riserMotor.setTargetPosition(500);
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    riserMotor.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5,()-> freightServo.setPosition(0))
                .UNSTABLE_addTemporalMarkerOffset(1.5,()-> freightServo.setPosition(1))
                .UNSTABLE_addTemporalMarkerOffset(2,()->{
                    riserMotor.setTargetPosition(0);
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    riserMotor.setPower(1);
                })
                .waitSeconds(2)
                .forward(0.5)
                .splineToSplineHeading(new Pose2d(7.5,-fieldHalf+hDiag, Math.toRadians(0)),Math.toRadians(270))
                .splineToConstantHeading(defaultPoseRed.vec(),Math.toRadians(270))
                .build();
        TrajectorySequence parkSequence = drive.trajectorySequenceBuilder(defaultPoseRed)
                .setReversed(true)
                .lineTo(new Vector2d(fieldHalf-hLength-20,-fieldHalf+hWidth))
                .splineToConstantHeading(new Vector2d(fieldHalf-hLength-20,-fieldHalf+hDiag+0.5),90)
                .lineTo(new Vector2d(fieldHalf-20-hLength,-fieldHalf+hWidth+27.5))
                .build();
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
                        currentState = AutoState.RETURNING_TO_DEFAULT_POS_FROM_HUB;
                        drive.followTrajectorySequenceAsync(returnFromHubSequence);
                    }
                case RETURNING_TO_DEFAULT_POS_FROM_HUB:
                    if(!drive.isBusy()){
                        currentState = AutoState.EN_ROUTE_TO_WAREHOUSE;
                        drive.followTrajectorySequenceAsync(firstFreightSequence);
                    }
            }
        }
    }
}
