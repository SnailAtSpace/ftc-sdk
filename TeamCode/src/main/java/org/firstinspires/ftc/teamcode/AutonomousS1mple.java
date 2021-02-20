package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name="Autonomous, but it's even worse somehow")

public class AutonomousS1mple extends OpMode {
    DcMotor FRmotor;DcMotor RRmotor;DcMotor FLmotor;DcMotor RLmotor;DcMotor Worm;Servo Grabber;
    ElapsedTime whenAreWe = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    @Override
    public void init(){
        FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");
        RRmotor = hardwareMap.get(DcMotor.class, "RRmotor");
        FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
        RLmotor = hardwareMap.get(DcMotor.class, "RLmotor");
        Worm = hardwareMap.get(DcMotor.class, "worm");
        Grabber = hardwareMap.get(Servo.class,"grabber");
        FLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    @SuppressLint("DefaultLocale")
    @Override
    public void start(){
        Grabber.scaleRange(0.22,0.66);
        Grabber.setPosition(0);
        whenAreWe.reset();
        FRmotor.setPower(1);
        RRmotor.setPower(1);
        FLmotor.setPower(1);
        RLmotor.setPower(1);
        while(whenAreWe.time()<=2400){}
        FRmotor.setPower(0);
        RRmotor.setPower(0);
        FLmotor.setPower(0);
        RLmotor.setPower(0);
    }
    @Override
    public void loop(){
        try {
            Thread.sleep(50);
        } catch (InterruptedException ignored) {
        }
    }
}
