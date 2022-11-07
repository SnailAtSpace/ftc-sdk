package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class CommonOpMode extends LinearOpMode {
    public SampleMecanumDrive drive;
    public DcMotorEx riserMotor;
    public OpenCvCamera webcam;
    public BingusPipeline pipeline;
    public BingusPipeline.RandomizationFactor duckPos = BingusPipeline.RandomizationFactor.UNDEFINED;
    final double restrictorCap = 1;
    double forward_axis, strafe_axis, turn_axis, riser_axis;
    double restrictor = restrictorCap;
    boolean previousFreight = false;
    int collector, previousCollector = 0;
    boolean freight;
    long upperArmLimit=1030;
    final double maxCollPower = 1;
    final double width = 330.29/25.4, length = 380.78/25.4, diag = Math.hypot(width,length);
    final double hWidth = width/2, hLength = length/2,hDiag=diag/2, fieldHalf = 70.5;
    public Pose2d startPoseRed = new Pose2d(10.5,-fieldHalf+hLength, Math.toRadians(270));
    public Pose2d warehousePoseRed = new Pose2d(fieldHalf-hLength-20,-fieldHalf+hWidth,Math.toRadians(0));
    public Pose2d defaultPoseRed = new Pose2d(9.5,-fieldHalf+hWidth,Math.toRadians(0));
    public void Initialize(HardwareMap hardwareMap, boolean isAuto) {
        drive = new SampleMecanumDrive(hardwareMap);
        riserMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "riserMotor");
        riserMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        riserMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (isAuto) {
            riserMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            pipeline = new BingusPipeline();
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
        else{
        }
    }

    public void safeSleep(int millis) {
        ElapsedTime localTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (localTime.time() < millis && opModeIsActive() && !isStopRequested()){}
    }


    public static double logifyInput(double input, int power) {
        return Math.abs(Math.pow(input, power)) * Math.signum(input);
    }

    @Deprecated
    public void ramIntoWall(boolean isRed){
        Pose2d startPose = drive.getPoseEstimate();
        drive.setWeightedDrivePower(new Pose2d(0, (isRed?-1:1)*0.2,0));
        drive.update();
        safeSleep(600);
        drive.setWeightedDrivePower(new Pose2d(0,0,0));
        drive.update();
        drive.setPoseEstimate(startPose);
    }
}