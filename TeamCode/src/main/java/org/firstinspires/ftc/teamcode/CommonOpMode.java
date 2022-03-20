package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class CommonOpMode extends LinearOpMode {
    public SampleMecanumDrive drive;
    public Servo freightServo;
    public DcMotorEx collectorMotor, riserMotor, carouselMotor;
    public OpenCvCamera webcam;
    public BingusPipeline pipeline;
    public RevTouchSensor armButton;
    public RevColorSensorV3 freightSensor;
    public BingusPipeline.RandomizationFactor duckPos = BingusPipeline.RandomizationFactor.UNDEFINED;
    final double restrictorCap = 1;
    double forward_axis, strafe_axis, turn_axis, riser_axis, carousel_axis;
    double restrictor = restrictorCap;
    boolean previousFreight = false;
    int collector, previousCollector = 0;
    boolean freight;
    long upperArmLimit=1030;
    final double maxCollPower = 1;
    final double width = 330.29/25.4, length = 380.78/25.4, diag = Math.hypot(width,length);
    final double hWidth = width/2, hLength = length/2,hDiag=diag/2, fieldHalf = 70.5;
    public Pose2d startPoseRed = new Pose2d(10.5,-fieldHalf+hLength, Math.toRadians(270));
    public Pose2d startPoseBlue = new Pose2d(12.5,fieldHalf-hLength, Math.toRadians(90));
    public Pose2d warehousePoseRed = new Pose2d(fieldHalf-hLength-20,-fieldHalf+hWidth,Math.toRadians(0));
    public Pose2d warehousePoseBlue = new Pose2d(fieldHalf-hLength-20,fieldHalf-hWidth,Math.toRadians(0));
    public Pose2d defaultPoseRed = new Pose2d(9.5,-fieldHalf+hWidth,Math.toRadians(0));
    public Pose2d defaultPoseBlue = new Pose2d(9.5,fieldHalf-hWidth,Math.toRadians(0));
    public void Initialize(HardwareMap hardwareMap, boolean isAuto) {
        drive = new SampleMecanumDrive(hardwareMap);
        collectorMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "collectorMotor");
        riserMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "riserMotor");
        freightServo = hardwareMap.get(Servo.class, "FreightServo");
        carouselMotor = (DcMotorEx) hardwareMap.get(DcMotor.class,"carouselMotor");
        armButton = hardwareMap.get(RevTouchSensor.class, "armButton");
        freightSensor = hardwareMap.get(RevColorSensorV3.class, "freightDetectionSensor");
        riserMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        riserMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        freightServo.scaleRange(0.12,0.72);
        freightServo.setPosition(0.7);
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
            freightServo.setPosition(1);
        }
    }

    public boolean hasElement(){
        return freightSensor.getDistance(DistanceUnit.MM)<40;
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