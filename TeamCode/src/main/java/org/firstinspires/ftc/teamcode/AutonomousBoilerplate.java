package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(preselectTeleOp = "W+M1", name = "AutoBoilerplate", group = "Main")
public class AutonomousBoilerplate extends CommonOpMode{
    private enum State {
        TRAVELLING_TO_FIRST_JUNCTION,
        GETTING_NEW_CONE,
        TRAVELLING_TO_SECOND_JUNCTION,
        IDLE
    }

    State currentState = State.IDLE, prevState = State.IDLE;
    @Override
    public void runOpMode() throws InterruptedException {
        Initialize(hardwareMap,true);
        while(opModeInInit()){
            duckPos = pipeline.ComposeTelemetry(telemetry);
        }
        if(isStopRequested())return;
        currentState = State.TRAVELLING_TO_FIRST_JUNCTION;
        while(opModeIsActive() && !isStopRequested()){
            switch (currentState){
                case TRAVELLING_TO_FIRST_JUNCTION:
                    // lol do something
                    break;
                case IDLE:
                    // lol do nothing
                    break;
                default:
                    break;
            }
            telemetry.addData("State: ", currentState.name());
            telemetry.addData("Position: ", "%.3f %.3f %.3f",drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading());
            telemetry.addData("Error: ","%.3f %.3f %.3f",drive.getLastError().getX(),drive.getLastError().getY(),drive.getLastError().getHeading());
            telemetry.addData("Riser: ", riserMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
