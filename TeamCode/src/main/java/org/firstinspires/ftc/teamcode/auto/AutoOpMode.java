package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.BingusPipeline;
import org.firstinspires.ftc.teamcode.CommonOpMode;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class AutoOpMode extends CommonOpMode {
    protected final double pi = Math.PI;
    final int maxCones = 1;

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
    int zone = 0;

    protected final Pose2d junctionPose = new Pose2d(new Vector2d(-95,760),Math.toRadians(-45));
    protected final Pose2d conePose = new Pose2d(-1700+125,300,pi);
    protected final Pose2d secondJunctionPose = new Pose2d(-600,238,1.5*pi);
    public final Pose2d coneLinePose = new Pose2d(-1400,280,pi);
    public final Pose2d closeParkPose = new Pose2d(300,280,1.5*pi),
                        centerParkPose = new Pose2d(900,280,1.5*pi),
                        edgeParkPose = new Pose2d(1500,280,1.5*pi);
    protected Pose2d startPose = new Pose2d(-856,fieldHalf-hWidth,Math.toRadians(-1));

    public void Initialize(HardwareMap hardwareMap, boolean mirroredX, boolean mirroredY) {
        super.Initialize(hardwareMap, mirroredX, mirroredY);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new BingusPipeline(mirroredX^mirroredY);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("error at "+ errorCode);
            }
        });
        telemetry.addLine("Waiting for start.");
        telemetry.addLine("Please calibrate starting position.");
        telemetry.update();
    }
}
