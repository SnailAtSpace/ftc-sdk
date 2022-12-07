package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public abstract class AutoOpMode extends CommonOpMode {
    public void Initialize(HardwareMap hardwareMap, boolean mirrored) {
        drive = new SampleMecanumDrive(hardwareMap, mirrored);
        super.Initialize(hardwareMap);
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
