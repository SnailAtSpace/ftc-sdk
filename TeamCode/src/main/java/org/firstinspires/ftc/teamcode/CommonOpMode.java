package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class CommonOpMode extends LinearOpMode {
    SampleMecanumDrive drive;
    Servo freightServo;
    DcMotorEx collectorMotor, riserMotor, carouselMotor, centerEncoder;
    OpenCvCamera webcam;
    BingusPipeline pipeline;
    RevTouchSensor armButton;
    RevColorSensorV3 freightSensor;
    public BingusPipeline.RandomizationFactor duckPos = BingusPipeline.RandomizationFactor.LEFT;
    final double restrictorCap = 0.9;
    double forward_axis, strafe_axis, turn_axis, riser_axis, carousel_axis;
    double restrictor = restrictorCap;
    boolean previousFreight = false;
    int collector, previousCollector = 0;
    boolean freight;
    long upperArmLimit=1030;
    final double maxCollPower = 0.6;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    final double width = 330.29/25.4, length = 380.78/25.4, diag = 503.8783666/25.4;
    final double hWidth = width/2, hLength = length/2,hDiag=diag/2, fieldHalf = 70.5;
    Pose2d startPoseRed = new Pose2d(10.5,-fieldHalf+hLength, Math.toRadians(270));
    Pose2d startPoseBlue = new Pose2d(12.5,fieldHalf-hLength, Math.toRadians(90));
    Pose2d defaultPoseRed = new Pose2d(7.5,-fieldHalf+hWidth,Math.toRadians(0));
    Pose2d defaultPoseBlue = new Pose2d(7.5,fieldHalf-hWidth,Math.toRadians(0));
    public void Initialize(HardwareMap hardwareMap, boolean isAuto) {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        collectorMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "collectorMotor");
        riserMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "riserMotor");
        freightServo = hardwareMap.get(Servo.class, "FreightServo");
        carouselMotor = (DcMotorEx) hardwareMap.get(DcMotor.class,"carouselMotor");
        armButton = hardwareMap.get(RevTouchSensor.class, "armButton");
        freightSensor = hardwareMap.get(RevColorSensorV3.class, "freightDetectionSensor");
        riserMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        collectorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        freightServo.scaleRange(0.15,0.72);
        if (isAuto) {
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

    @Deprecated
    public static double logifyInput(double input, int power) {
        return Math.abs(Math.pow(input, power)) * Math.signum(input);
    }

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