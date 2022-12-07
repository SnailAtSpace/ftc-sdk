package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(preselectTeleOp = "W+M1", name = "AutoBoilerplate", group = "Main")
public class AutonomousBoilerplate extends AutoOpMode{
    private enum State {
        NAVIGATING_TO_FIRST_JUNCTION,
        PLACING_CONE,
        GETTING_NEW_CONE,
        NAVIGATING_TO_SECOND_JUNCTION,
        IDLE
    }

    State currentState = State.IDLE;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        Initialize(hardwareMap,false);
        drive.setPoseEstimate(startPose);
        TrajectorySequence firstJunctionSequence = constructPathToFirstJunction();
        TrajectorySequence getConeSequence = constructPathToNewCone();
        riserServo.setPosition(1);
        riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(opModeInInit()){
            //duckPos = pipeline.ComposeTelemetry(telemetry);
        }
        if(isStopRequested())return;
        currentState = State.NAVIGATING_TO_FIRST_JUNCTION;
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
                    currentState = State.GETTING_NEW_CONE;
                    drive.followTrajectorySequenceAsync(getConeSequence);
                    break;
                case GETTING_NEW_CONE:
                    if(!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                case NAVIGATING_TO_SECOND_JUNCTION:
                    //lol
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
