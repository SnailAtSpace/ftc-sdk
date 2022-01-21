package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class CommonOpMode extends LinearOpMode {
    DcMotorEx[] movementMotors = new DcMotorEx[4];
    Servo freightServo;
    DcMotor collectorMotor, riserMotor;
    OpenCvCamera webcam;
    BingusPipeline pipeline;
    Boolean ExecuteFlag;
    BNO055IMU imu;

    public enum Color {
        BLUE,
        RED
    }

    Color color;
    public BingusPipeline.RandomizationFactor ringData = BingusPipeline.RandomizationFactor.LEFT;
    final int LogPower = 3;
    double restrictor = 1;
    double forward_axis, strafe_axis, turn_axis, worm_axis, riser_axis;
    boolean previous_collector = false,previous_freight = false;
    boolean collector, freight;
    //TODO: Change safeArmLimit according to the robot        VVVV
    long upperArmLimit=1080, lowerArmLimit=10, safeArmLimit = 400;
    final double maxCollPower = 0.66;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public void Initialize(HardwareMap hardwareMap, boolean isAuto) {
        movementMotors[0] = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftFrontMotor");
        movementMotors[1] = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftRearMotor");
        movementMotors[2] = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightRearMotor");
        movementMotors[3] = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightFrontMotor");
        collectorMotor = hardwareMap.get(DcMotor.class, "collectorMotor");
        riserMotor = hardwareMap.get(DcMotor.class, "riserMotor");
        freightServo = hardwareMap.get(Servo.class, "FreightServo");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        for (DcMotor motor : movementMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        riserMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        riserMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        freightServo.scaleRange(0,0.66);
        movementMotors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        movementMotors[3].setDirection(DcMotorSimple.Direction.REVERSE);
        if (isAuto) {
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
        imuInitialization();
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
}