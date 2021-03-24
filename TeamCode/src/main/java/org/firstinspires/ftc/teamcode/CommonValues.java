package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.reflect.*;
public class CommonValues{
    DcMotor FRmotor;
    DcMotor RRmotor;
    DcMotor FLmotor;
    DcMotor RLmotor;
    DcMotor Flywheel;
    DcMotor Worm;
    Servo Grabber;
    Servo Pushrod;
    DcMotor Collector;
    Method sleep;
    public void Initialize(HardwareMap hardwareMap) {
        FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");
        RRmotor = hardwareMap.get(DcMotor.class, "RRmotor");
        FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
        RLmotor = hardwareMap.get(DcMotor.class, "RLmotor");
        Flywheel = hardwareMap.get(DcMotor.class, "FWmotor");
        Worm = hardwareMap.get(DcMotor.class, "Wmotor");
        Grabber = hardwareMap.get(Servo.class, "Gservo");
        Pushrod = hardwareMap.get(Servo.class, "Pservo");
        Collector = hardwareMap.get(DcMotor.class, "Cmotor");
        FRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Worm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.sleep = sleep;
    }

    public void MoveByMillimetres(float millis, int direction){
        //direction counted from 0, being backwards, counterclockwise
        //0=backward, 1=right, 2=forward, 3=left
        ElapsedTime localTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (localTime.time() <= millis * 1.135) { //what the fuck am i doing
            RLmotor.setPower(Math.signum((direction - 1) * 2 - 1));
            RRmotor.setPower(Math.signum(direction % 3 * 2 - 1));
            FLmotor.setPower(Math.signum(direction % 3 * 2 - 1));
            FRmotor.setPower(Math.signum((direction - 1) * 2 - 1));
        }
        RLmotor.setPower(0);
        RRmotor.setPower(0);
        FLmotor.setPower(0);
        FRmotor.setPower(0);
        try {
            sleep(250);
        } catch (InterruptedException ignored) {
        }
    }

    public void DeployArm() throws InterruptedException {
        Worm.setPower(-1);
        sleep(2100);
        Worm.setPower(0);
    }

    private void sleep(long i) throws InterruptedException {
        Thread.sleep(i);
    }

    public void LaunchSeveralRings(int amount) {
        ElapsedTime localTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        Flywheel.setPower(1);
        while (localTime.time() <= 3000) {
        }
        Pushrod.setPosition(1);
        while (localTime.time() <= 3100) {
        }
        Pushrod.setPosition(0);
        for (int i = 0; i <= amount--; i++) {
            localTime.reset();
            while (localTime.time() <= 2000) {
            }
            Pushrod.setPosition(1);
            while (localTime.time() <= 2100) {
            }
            Pushrod.setPosition(0);
        }
        Flywheel.setPower(0);
    }

    public void TurnBySeconds(int millis, int direction) throws InterruptedException {
        ElapsedTime localTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (localTime.time() <= millis) { //what the fuck am i doing
            RLmotor.setPower(1 - direction * 2);
            RRmotor.setPower(direction * 2 - 1);
            FLmotor.setPower(1 - direction * 2);
            FRmotor.setPower(direction * 2 - 1);
        }
        RLmotor.setPower(0);
        RRmotor.setPower(0);
        FLmotor.setPower(0);
        FRmotor.setPower(0);
        sleep(100);
    }
}
