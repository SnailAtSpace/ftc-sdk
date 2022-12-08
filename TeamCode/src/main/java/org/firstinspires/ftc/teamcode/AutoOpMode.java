package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public abstract class AutoOpMode extends CommonOpMode {
    protected final double pi = Math.PI;

    protected final Pose2d startPose = new Pose2d(-600,fieldHalf-hWidth,0);
    protected TrajectorySequence firstJunctionSequence;
    protected TrajectorySequence getConeSequence;
    protected TrajectorySequence nudgePathSequence;
    protected TrajectorySequence coneLineSequence;

    protected final Pose2d junctionPose = new Pose2d(new Vector2d(-310,900)
            .plus(new Vector2d(Math.hypot(300,300)-hLength-50,0).rotated(Math.toRadians(-45)))
            .plus(new Vector2d(-55,0).rotated(Math.toRadians(45))),Math.toRadians(-40));

    public void Initialize(HardwareMap hardwareMap, boolean mirrored) {
        super.Initialize(hardwareMap, mirrored);
        drive.setPoseEstimate(startPose);
        firstJunctionSequence = pathToFirstJunction();
        coneLineSequence = pathToConeLine();
        getConeSequence = pathToCones();
        nudgePathSequence = nudgePath();
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

    public TrajectorySequence pathToFirstJunction(){
        return drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(1400, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    riserMotor.setTargetPosition(armExtensionToEncoderTicks(900));
                    riserMotor.setPower(1);
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineToConstantHeading(new Vector2d(-400,fieldHalf-500),3*pi/2.0f)
                .splineToSplineHeading(junctionPose,Math.toRadians(-45))
                .build();
    }

    public TrajectorySequence pathToConeLine(){
        return drive.trajectorySequenceBuilder(junctionPose)
                .back(50)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    riserMotor.setTargetPosition(0);
                    riserMotor.setPower(0.75);
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineToSplineHeading(new Pose2d(-300,600,3*pi/2.0f),3*pi/2.0f)
                .splineToSplineHeading(new Pose2d(-600,300,pi),pi)
                .splineToConstantHeading(new Vector2d(-1500,295),pi)
                .build();
    }

    public TrajectorySequence nudgePath(){
        return drive.trajectorySequenceBuilder(new Pose2d(-1500,280))
                .lineToLinearHeading(new Pose2d(-1500,300,pi))
                .build();
    }

    public TrajectorySequence pathToCones(){
        return drive.trajectorySequenceBuilder(new Pose2d(-1500,300,pi))
                .lineToConstantHeading(new Vector2d(-1700,300))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()->{
                    riserMotor.setTargetPosition(armExtensionToEncoderTicks(100));
                    riserMotor.setPower(1);
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .UNSTABLE_addTemporalMarkerOffset(0,()->riserServo.setPosition(1))
                .build();
    }

    public int armExtensionToEncoderTicks(double h){
        return (int) -(h/975*2839);
    }
}
