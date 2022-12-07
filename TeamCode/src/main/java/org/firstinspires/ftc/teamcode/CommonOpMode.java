package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;

public abstract class CommonOpMode extends LinearOpMode {

    // constants
    final double restrictorCap = 1;
    final double width = 380, length = 330, diag = Math.hypot(width,length);
    final double hWidth = width/2, hLength = length/2,hDiag=diag/2, fieldHalf = 1790.7;
    long upperArmLimit=2840;

    // webcam and rand-related information
    public OpenCvCamera webcam;
    public BingusPipeline pipeline;
    public BingusPipeline.RandomizationFactor duckPos = BingusPipeline.RandomizationFactor.UNDEFINED;

    // actuators
    public SampleMecanumDrive drive;
    public DcMotorEx riserMotor;
    public Servo riserServo;

    // i/o
    double forward_axis, strafe_axis, turn_axis, riser_axis; // analog inputs
    boolean riserArm; // digital inputs
    boolean pRiserArm; // last-loop digital inputs
    double riserPos;
    double restrictor = restrictorCap;

    public void Initialize(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        riserMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "riserMotor");
        riserServo = hardwareMap.get(Servo.class, "riserServo");
        riserMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        riserMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        riserServo.scaleRange(0,0.2);
    }

    public void safeSleep(int millis) {
        ElapsedTime localTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (localTime.time() < millis && opModeIsActive() && !isStopRequested()){}
    }
}