package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;

public abstract class CommonOpMode extends LinearOpMode {

    // constants
    final public double restrictorCap = 1;
    final public double width = 380, length = 330, diag = Math.hypot(width,length);
    final public double hWidth = width/2, hLength = length/2,hDiag=diag/2, fieldHalf = 1800;
    long upperArmLimit=2840;

    // webcam and rand-related information
    public OpenCvCamera webcam;
    public BingusPipeline pipeline;
    public BingusPipeline.RandomizationFactor duckPos = BingusPipeline.RandomizationFactor.UNDEFINED;

    // actuators
    public SampleMecanumDrive drive;
    public DcMotorEx riserMotor;
    public Servo riserServo;

    // sensors
    public Rev2mDistanceSensor distanceSensor;
    public RevColorSensorV3 lineSensor;
    public RevTouchSensor armLimiter;

    // i/o
    double forward_axis, strafe_axis, turn_axis, riser_axis; // analog inputs
    boolean riserArm; // digital inputs
    boolean pRiserArm; // last-loop digital inputs
    double riserPos;
    double restrictor = restrictorCap;

    public void Initialize(HardwareMap hardwareMap, boolean mirrored) {
        drive = new SampleMecanumDrive(hardwareMap, mirrored);
        riserMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "riserMotor");
        riserServo = hardwareMap.get(Servo.class, "riserServo");
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "DistanceSensor");
        lineSensor = hardwareMap.get(RevColorSensorV3.class, "LineSensor");
        riserMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        riserMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLimiter = hardwareMap.get(RevTouchSensor.class, "armLimiter");
        lineSensor.initialize();
        riserServo.scaleRange(0.06,0.40);
    }

    public void Initialize(HardwareMap hardwareMap){
        this.Initialize(hardwareMap, false);
    }

    public void safeSleep(int millis) {
        ElapsedTime localTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (localTime.time() < millis && opModeIsActive() && !isStopRequested()){}
    }

    public int armExtensionToEncoderTicks(double h){
        return (int) -(h/975*2839);
    }

    public double encoderTicksToArmExtension(long t){
        return -t/2879.0*975.0;
    }
}