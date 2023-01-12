package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommonOpMode;

public abstract class AutoOpMode extends CommonOpMode {
    protected final double pi = Math.PI;
    final int maxCones = 5;

    public enum State {
        NAVIGATING_TO_FIRST_JUNCTION,
        PLACING_CONE,
        NAVIGATING_TO_CONE_STACK,
        SEEKING_CONE_LINE,
        COLLECTING_CONE,
        PREPARING_FOR_DEPARTURE,
        NAVIGATING_TO_SECOND_JUNCTION,
        PARKING,
        IDLE
    }

    State currentState = State.IDLE;
    ElapsedTime timer = new ElapsedTime();

    protected final Pose2d junctionPose = new Pose2d(new Vector2d(-325,895)
            .plus(new Vector2d(Math.hypot(300,300)-hLength-28,0).rotated(Math.toRadians(-45)))
            ,Math.toRadians(-45));
    protected final Pose2d conePose = new Pose2d(-1700+155,300,pi);
    protected final Pose2d secondJunctionPose = new Pose2d(-610,208,1.5*pi);

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
