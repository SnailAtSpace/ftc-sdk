package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomous, but it's bad", group = "")

public class AutonomousTest extends OpMode {
    ElapsedTime whenAreWe = new ElapsedTime();;
    DcMotor FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");                    //Hardware mapping and declaration of devices
    DcMotor RRmotor = hardwareMap.get(DcMotor.class, "RRmotor");
    DcMotor FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
    DcMotor RLmotor = hardwareMap.get(DcMotor.class, "RLmotor");
    DcMotor Worm = hardwareMap.get(DcMotor.class, "worm");
    Servo Grabber = hardwareMap.get(Servo.class,"grabber");
    @Override
    public void init(){
        Grabber.scaleRange(0.22,0.66);
        Grabber.setPosition(1);
    }
    @Override
    public void loop(){
        whenAreWe.reset();
        double for_axis=0;
        double strafe_axis=0;
        double turn_axis=0;
        if(whenAreWe.time()<=) {}
        FRmotor.setPower(for_axis + strafe_axis + turn_axis);                                   //wheel motor movement
        RRmotor.setPower(for_axis - strafe_axis + turn_axis);
        FLmotor.setPower(-for_axis + strafe_axis + turn_axis);
        RLmotor.setPower(-for_axis - strafe_axis + turn_axis);
    }
}
