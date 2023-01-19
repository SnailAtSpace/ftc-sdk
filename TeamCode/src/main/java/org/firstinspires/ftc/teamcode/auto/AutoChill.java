package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public abstract class AutoChill extends AutoOpMode{
    public TrajectorySequence firstJunctionSequence;
    public TrajectorySequence[] parkingSequence = new TrajectorySequence[3];
    boolean mirrored = false;
    public void runOpMode(boolean mirrored) throws InterruptedException {
        super.Initialize(hardwareMap, mirrored, false);
        this.mirrored = mirrored;
        firstJunctionSequence = pathToFirstJunction();
        parkingSequence[0] = pathToParking1();
        parkingSequence[1] = pathToParking2();
        parkingSequence[2] = pathToParking3();
        riserServo.setPosition(1);
        riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addLine("Initialization complete; robot ready for match start.");
        while(opModeInInit()){
            zone = pipeline.ComposeTelemetry(telemetry);
        }
        if(isStopRequested())return;
        currentState = State.NAVIGATING_TO_FIRST_JUNCTION;
        drive.setCorrectedPoseEstimate(startPose);
        drive.followTrajectorySequenceAsync(firstJunctionSequence);
        drive.update();
        while (!isStopRequested() && opModeIsActive()){
            switch (currentState){
                case NAVIGATING_TO_FIRST_JUNCTION:
                    if(!drive.isBusy()){
                        riserServo.setPosition(0);
                        currentState = State.PARKING;
                        drive.setCorrectedPoseEstimate(junctionPose);
                        drive.followTrajectorySequenceAsync(parkingSequence[(mirrored?2:0)+(mirrored?-1:1)*(zone-1)]);
                    }
                    break;
                case PARKING:
                    if(!drive.isBusy()){
                        currentState = State.IDLE;
                    }
                    break;
                default:
                    idle();
            }
            drive.update();
        }
    }


    public abstract TrajectorySequence pathToFirstJunction();

    public abstract TrajectorySequence pathToParking1();

    public abstract TrajectorySequence pathToParking2();

    public abstract TrajectorySequence pathToParking3();
}
