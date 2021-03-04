package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp
public class prikol extends LinearOpMode {
    DcMotor FRmotor;DcMotor RRmotor;DcMotor FLmotor;DcMotor RLmotor;DcMotor Worm;
    Servo Grabber;DcMotor Flywheel;Servo Pushrod;
    @Override
    public void runOpMode(){
        FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");                        //Hardware mapping and declaration of devices
        RRmotor = hardwareMap.get(DcMotor.class, "RRmotor");
        FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
        RLmotor = hardwareMap.get(DcMotor.class, "RLmotor");
        Flywheel = hardwareMap.get(DcMotor.class, "FWmotor");
        Worm = hardwareMap.get(DcMotor.class, "Wmotor");
        Grabber = hardwareMap.get(Servo.class,"Gservo");
        Pushrod = hardwareMap.get(Servo.class,"Pservo");
        Flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        FRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        waitForStart();
        if(opModeIsActive()){
            while(opModeIsActive()){
                MoveByMillimetres(-1+(gamepad1.dpad_down?1:0)+(gamepad1.dpad_right?2:0)+(gamepad1.dpad_up?3:0)+(gamepad1.dpad_left?4:0));
            }
        }
    }
    public void MoveByMillimetres(int direction){
        //direction counted from 0, being backwards, counterclockwise
        //0=backward, 1=left, 2=forward, 3=right
        if(direction>=0){
            RLmotor.setPower(Math.signum((direction-1)*2-1));
            RRmotor.setPower(Math.signum(direction%3*2-1));
            FLmotor.setPower(Math.signum(direction%3*2-1));
            RLmotor.setPower(Math.signum((direction-1)*2-1));
        }
    }
}
