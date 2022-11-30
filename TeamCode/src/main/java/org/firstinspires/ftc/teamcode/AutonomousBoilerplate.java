package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(preselectTeleOp = "W+M1", name = "AutoBoilerplate", group = "Main")
public class AutonomousBoilerplate extends CommonOpMode{
    final double pi = Math.PI;
    private final Pose2d startPose = new Pose2d(-600,fieldHalf-hWidth,0);
    private final Pose2d junctionPose = new Pose2d(new Vector2d(-300,900)
            .plus(new Vector2d(Math.hypot(300,300)-hLength-50,0).rotated(Math.toRadians(-45))),Math.toRadians(-45));

    private enum State {
        TRAVELLING_TO_FIRST_JUNCTION,
        PLACING_CONE,
        GETTING_NEW_CONE,
        TRAVELLING_TO_SECOND_JUNCTION,
        IDLE
    }

    State currentState = State.IDLE, prevState = State.IDLE;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        Initialize(hardwareMap,true);
        drive.setPoseEstimate(startPose);
        TrajectorySequence firstJunctionSequence = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-300,fieldHalf-600),3*pi/2.0f)
                .splineToSplineHeading(junctionPose,Math.toRadians(-45))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->riserServo.setPosition(1))
                .build();
        TrajectorySequence getConeSequence = drive.trajectorySequenceBuilder(junctionPose)
                .back(30)
                .splineToSplineHeading(new Pose2d(-300,600,3*pi/2.0f),3*pi/2.0f)
                .splineToSplineHeading(new Pose2d(-600,300,pi),pi)
                .splineToConstantHeading(new Vector2d(-1200,300),pi)
                .build();
        while(opModeInInit()){
            //duckPos = pipeline.ComposeTelemetry(telemetry);
        }
        if(isStopRequested())return;
        currentState = State.TRAVELLING_TO_FIRST_JUNCTION;
        drive.followTrajectorySequenceAsync(firstJunctionSequence);
        while(opModeIsActive() && !isStopRequested()){
            switch (currentState){
                case TRAVELLING_TO_FIRST_JUNCTION:
                    // lol do something
                    if(!drive.isBusy()) {
                        currentState = State.PLACING_CONE;
                        timer.reset();
                    }
                    break;
                case PLACING_CONE:
                    if(timer.time()>3){
                        currentState = State.GETTING_NEW_CONE;
                        drive.followTrajectorySequenceAsync(getConeSequence);
                    }
                    break;
                case GETTING_NEW_CONE:
                    if(!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
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

    public TrajectorySequence constructPathToFirstJunction(){
        return drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-300,-fieldHalf+600),Math.PI/2.0f)
                .splineToSplineHeading(junctionPose,Math.toRadians(135))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->riserServo.setPosition(1))
                .build();
    }

    public TrajectorySequence constructPathToNewCone(){
        return drive.trajectorySequenceBuilder(junctionPose)
                .splineToSplineHeading(new Pose2d(300,600,pi/2.0f),pi/2.0f)
                .splineToSplineHeading(new Pose2d(600,300,0),0)
                .splineToConstantHeading(new Vector2d(900,300),0)
                .build();
    }
}
