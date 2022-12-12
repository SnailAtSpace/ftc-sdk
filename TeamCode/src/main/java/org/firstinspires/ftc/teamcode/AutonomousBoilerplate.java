package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(preselectTeleOp = "W+M1", name = "AutoBoilerplate", group = "Main")
public class AutonomousBoilerplate extends AutoOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Initialize(hardwareMap,false);
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
                        riserMotor.setTargetPosition(armExtensionToEncoderTicks(920));
                        riserMotor.setPower(1);
                        riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        timer.reset();
                        currentState = State.PREPARING_FOR_DEPARTURE;
                    }
                    break;
                case PREPARING_FOR_DEPARTURE:
                    if(timer.seconds()>0.4){
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
            telemetry.addData("Riser: ", riserMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
