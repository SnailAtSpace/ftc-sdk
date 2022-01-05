package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class CommonOpMode extends LinearOpMode {

    DcMotor[] movementMotors = new DcMotor[4];
    OpenCvCamera webcam;
    BingusPipeline pipeline;
    Boolean ExecuteFlag;
    BNO055IMU imu;
    BingusPipeline.StartLine side=BingusPipeline.StartLine.RIGHT;
    public enum Color {
        BLUE,
        RED
    }
    Color color;
    public BingusPipeline.RandomizationFactor ringData=BingusPipeline.RandomizationFactor.ZERO;
    final int LogPower = 3;
    double forward_axis,strafe_axis,turn_axis,worm_axis;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public void Initialize(HardwareMap hardwareMap, boolean isAuto, BingusPipeline.StartLine side) {
        movementMotors[0] = hardwareMap.get(DcMotor.class, "FRmotor");
        movementMotors[1] = hardwareMap.get(DcMotor.class, "RRmotor");
        movementMotors[2] = hardwareMap.get(DcMotor.class, "FLmotor");
        movementMotors[3] = hardwareMap.get(DcMotor.class, "RLmotor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        for (DcMotor motor:movementMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        movementMotors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        movementMotors[3].setDirection(DcMotorSimple.Direction.REVERSE);
        if(isAuto) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            pipeline = new BingusPipeline();
            webcam.setPipeline(pipeline);
            pipeline.setSide(side);
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                }
            });
            telemetry.addLine("Waiting for start.");
            telemetry.addLine("Please calibrate starting position.");
            telemetry.update();
        }
        imuInitialization();
        this.side = side;
    }

    public void safeSleep(int millis){
        ElapsedTime localTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(localTime.time()<millis&&opModeIsActive())idle();
    }

    public double rpmToTps(double rpm){
        return rpm*28/60.0;
    }

    public static double logarithmifyInput(double input, int power) {
        return Math.abs(Math.pow(input,power))*Math.signum(input);
    }

    public void imuInitialization(){
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "calib.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.mode = BNO055IMU.SensorMode.NDOF;
        imu.initialize(parameters);
    }
}