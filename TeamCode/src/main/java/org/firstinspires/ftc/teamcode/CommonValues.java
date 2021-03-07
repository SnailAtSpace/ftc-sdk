//package org.firstinspires.ftc.teamcode;
//
//import android.content.Context;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.*;
//import java.util.stream.Collector;
//
//public abstract class CommonValues extends LinearOpMode {
//    static final DcMotor FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");                        //Hardware mapping and declaration of devices
//    static final DcMotor RRmotor = hardwareMap.get(DcMotor.class, "RRmotor");
//    static final DcMotor FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
//    static final DcMotor RLmotor = hardwareMap.get(DcMotor.class, "RLmotor");
//    static final DcMotor Flywheel = hardwareMap.get(DcMotor.class, "FWmotor");
//    static final DcMotor Worm = hardwareMap.get(DcMotor.class, "Wmotor");
//    static final Servo Grabber = hardwareMap.get(Servo.class,"Gservo");
//    static final Servo Pushrod = hardwareMap.get(Servo.class,"Pservo");
//    static final DcMotor Collector = hardwareMap.get(DcMotor.class,"Cmotor");
//    public static void Initialize(){
//        FRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        RRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        RLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        Worm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//    public void MoveByMillimetres(float millis,int direction) throws InterruptedException {
//        //direction counted from 0, being backwards, counterclockwise
//        //0=backward, 1=right, 2=forward, 3=left
//        ElapsedTime localTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//        while(localTime.time()<=millis*1.135) { //what the fuck am i doing
//            CommonValues.RLmotor.setPower(Math.signum((direction-1)*2-1));
//            CommonValues.RRmotor.setPower(Math.signum(direction%3*2-1));
//            CommonValues.FLmotor.setPower(Math.signum(direction%3*2-1));
//            CommonValues.FRmotor.setPower(Math.signum((direction-1)*2-1));
//        }
//        CommonValues.RLmotor.setPower(0);
//        CommonValues.RRmotor.setPower(0);
//        CommonValues.FLmotor.setPower(0);
//        CommonValues.FRmotor.setPower(0);
//        sleep(250);
//    }
//    public void DeployArm() throws InterruptedException {
//        CommonValues.Worm.setPower(-1);
//        sleep(2100);
//        CommonValues.Worm.setPower(0);
//    }
//
//    private void sleep(int i) throws InterruptedException {
//        Thread.sleep(i);
//    }
//
//    public void LaunchSeveralRings(int amount){
//        ElapsedTime localTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//        CommonValues.Flywheel.setPower(1);
//        while (localTime.time() <= 3000) {}
//        CommonValues.Pushrod.setPosition(1);
//        while (localTime.time() <= 3100) {}
//        CommonValues.Pushrod.setPosition(0);
//        for(int i=0;i<=amount--;i++){
//            localTime.reset();
//            while (localTime.time() <= 2000) {}
//            CommonValues.Pushrod.setPosition(1);
//            while (localTime.time() <= 2100) {}
//            CommonValues.Pushrod.setPosition(0);
//        }
//        CommonValues.Flywheel.setPower(0);
//    }
//    public void TurnBySeconds(int millis,int direction) throws InterruptedException {
//        ElapsedTime localTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//        while(localTime.time()<=millis) { //what the fuck am i doing
//            CommonValues.RLmotor.setPower(1-direction*2);
//            CommonValues.RRmotor.setPower(direction*2-1);
//            CommonValues.FLmotor.setPower(1-direction*2);
//            CommonValues.FRmotor.setPower(direction*2-1);
//        }
//        CommonValues.RLmotor.setPower(0);
//        CommonValues.RRmotor.setPower(0);
//        CommonValues.FLmotor.setPower(0);
//        CommonValues.FRmotor.setPower(0);
//        sleep(100);
//    }
//}
