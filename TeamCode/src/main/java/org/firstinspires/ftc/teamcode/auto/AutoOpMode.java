package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommonOpMode;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public abstract class AutoOpMode extends CommonOpMode {
    protected final double pi = Math.PI;

    public enum State {
        NAVIGATING_TO_FIRST_JUNCTION,
        PLACING_CONE,
        NAVIGATING_TO_CONE_STACK,
        SEEKING_CONE_LINE,
        ALIGNING_WITH_CONE_LINE,
        COLLECTING_CONE,
        PREPARING_FOR_DEPARTURE,
        NAVIGATING_TO_SECOND_JUNCTION,
        IDLE
    }

    State currentState = State.IDLE;
    ElapsedTime timer = new ElapsedTime();

    protected final Pose2d junctionPose = new Pose2d(new Vector2d(-310,900)
            .plus(new Vector2d(Math.hypot(300,300)-hLength-30,0).rotated(Math.toRadians(-45)))
            .plus(new Vector2d(-50,0).rotated(Math.toRadians(45))),Math.toRadians(-40));

    public void Initialize(HardwareMap hardwareMap, boolean mirrored) {
        super.Initialize(hardwareMap, mirrored);
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        pipeline = new BingusPipeline();
//        webcam.setPipeline(pipeline);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                telemetry.addLine("error at "+ errorCode);
//            }
//        });
        telemetry.addLine("Waiting for start.");
        telemetry.addLine("Please calibrate starting position.");
        telemetry.update();
    }
}
