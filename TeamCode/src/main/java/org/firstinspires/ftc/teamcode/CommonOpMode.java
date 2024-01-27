package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;

public abstract class CommonOpMode extends LinearOpMode {

    // constants
    final public double restrictorCap = 1;
    final public double width = 380, length = 330, diag = Math.hypot(width, length);
    final public double hWidth = width / 2, hLength = length / 2, hDiag = diag / 2, fieldHalf = 1800;

    // actuators
    public MecanumDrive drive;

    // webcam and rand-related information
    public OpenCvCamera webcam;
    public BingusPipeline pipeline;
    public int rand = 0;
    public DuplexMotor riserMotor, collectorMotor;
    public Servo riserServoA, riserServoB, pusherServo;
    // TODO: изменить максимальную высоту подъёма в соответствии
    long upperArmLimit = 2840;

    // sensors
    public Rev2mDistanceSensor distanceSensor;
    public RevColorSensorV3 lineSensor;
    public RevTouchSensor armLimiter;

    // i/o
    double forward_axis, strafe_axis, turn_axis, riser_axis; // analog inputs
    boolean riserArm, pusher; // digital inputs
    boolean pRiserArm, ppusher; // last-loop digital inputs
    double riserPos;
    double restrictor = restrictorCap;

    public void Initialize(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        riserMotor = new DuplexMotor((DcMotorEx) hardwareMap.get(DcMotor.class, "riserMotorA"),
                (DcMotorEx) hardwareMap.get(DcMotor.class, "riserMotorB"));
        collectorMotor = new DuplexMotor((DcMotorEx) hardwareMap.get(DcMotor.class, "collectorMotorA"),
                (DcMotorEx) hardwareMap.get(DcMotor.class, "collectorMotorB"));
        riserServoA = hardwareMap.get(Servo.class, "riserServoA");
        riserServoB = hardwareMap.get(Servo.class, "riserServoB");
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "DistanceSensor");
        //lineSensor = hardwareMap.get(RevColorSensorV3.class, "LineSensor");
        riserMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        riserMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLimiter = hardwareMap.get(RevTouchSensor.class, "armLimiter");
        //lineSensor.initialize();

        //TODO: отрегулировать сервомоторы
        riserServoA.scaleRange(0.06, 0.40);
        riserServoB.scaleRange(0.06, 0.40);
        pusherServo.scaleRange(0, 1);
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