package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class CommonOpMode extends LinearOpMode {
    DcMotorEx[] movementMotors = new DcMotorEx[4];
    Servo freightServo;
    DcMotorEx collectorMotor, riserMotor, carouselMotor;
    OpenCvCamera webcam;
    BingusPipeline pipeline;
    Boolean ExecuteFlag;
    BNO055IMU imu;

    public enum Color {
        BLUE,
        RED
    }

    Color color;
    public BingusPipeline.RandomizationFactor duckPos = BingusPipeline.RandomizationFactor.LEFT;
    final int LogPower = 3;
    double restrictor = 1;
    double forward_axis, strafe_axis, turn_axis, worm_axis, riser_axis, carousel_axis;
    boolean previous_collector = false,previous_freight = false;
    boolean collector, freight;
    //TODO: Change safeArmLimit according to the robot        VVVV
    long upperArmLimit=1100, lowerArmLimit=5, safeArmLimit = 400;
    final double maxCollPower = 0.5;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    final double width = 330.29/25.4, length = 430/25.4, diag = 542.3/25.4;
    final double hWidth = width/2, hLength = length/2,hDiag=diag/2, fieldHalf = 70.5;
    Pose2d startPoseRed = new Pose2d(7.5,-fieldHalf+hLength, Math.toRadians(270));
    Pose2d startPoseBlue = new Pose2d(15,fieldHalf-hLength, Math.toRadians(90));
    Pose2d defaultPoseRed = new Pose2d(7.5,-fieldHalf+hWidth,Math.toRadians(180));
    Pose2d defaultPoseBlue = new Pose2d(7.5,fieldHalf-hWidth,Math.toRadians(0));
    public void Initialize(HardwareMap hardwareMap, boolean isAuto) {
        movementMotors[0] = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftFrontMotor");
        movementMotors[1] = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftRearMotor");
        movementMotors[2] = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightRearMotor");
        movementMotors[3] = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightFrontMotor");
        collectorMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "collectorMotor");
        riserMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "riserMotor");
        freightServo = hardwareMap.get(Servo.class, "FreightServo");
        carouselMotor = (DcMotorEx) hardwareMap.get(DcMotor.class,"carouselMotor");
        riserMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        freightServo.scaleRange(0,0.75);
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
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            movementMotors[2].setDirection(DcMotorSimple.Direction.REVERSE);
            movementMotors[3].setDirection(DcMotorSimple.Direction.REVERSE);
            for (DcMotor motor : movementMotors) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            imuInitialization();
        }
    }

    public void safeSleep(int millis) {
        ElapsedTime localTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (localTime.time() < millis && opModeIsActive()) idle();
    }

    public double rpmToTps(double rpm) {
        return rpm * 28 / 60.0;
    }

    public static double logifyInput(double input, int power) {
        return Math.abs(Math.pow(input, power)) * Math.signum(input);
    }

    public void imuInitialization() {
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "calib.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.mode = BNO055IMU.SensorMode.NDOF;
        imu.initialize(parameters);
    }
    public void ramIntoWall(boolean isRed){
        movementMotors[0].setPower(-0.2);
        movementMotors[1].setPower(0.2);
        movementMotors[2].setPower(-0.2);
        movementMotors[3].setPower(0.2);
        safeSleep(400);
        movementMotors[0].setPower(0);
        movementMotors[1].setPower(0);
        movementMotors[2].setPower(0);
        movementMotors[3].setPower(0);
    }
}