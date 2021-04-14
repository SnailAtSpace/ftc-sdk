package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Bingus Controller Test")
public class ControllerTest extends LinearOpMode {
  @Override
  public void runOpMode() {
    DcMotor FRmotor, RRmotor, FLmotor, RLmotor, Worm, Flywheel, Collector;
    Servo Grabber, Pushrod;
    FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");                        //Hardware mapping and declaration of devices
    RRmotor = hardwareMap.get(DcMotor.class, "RRmotor");
    FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
    RLmotor = hardwareMap.get(DcMotor.class, "RLmotor");
    Flywheel = hardwareMap.get(DcMotor.class, "FWmotor");
    Worm = hardwareMap.get(DcMotor.class, "Wmotor");
    Grabber = hardwareMap.get(Servo.class,"Gservo");
    Pushrod = hardwareMap.get(Servo.class,"Pservo");
    Collector = hardwareMap.get(DcMotor.class,"Cmotor");
    FRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    FLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);//Initialization phase
    Worm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    final int LogPower=3;
    boolean grab = false;
    boolean push = false;
    boolean collector = false;
    boolean flywheel = false;
    boolean prevgrab;
    boolean prevpush;
    boolean prevfly;
    boolean prevcoll;
    waitForStart();
    if (opModeIsActive()) {                                                                         //Pre-run phase
      Grabber.scaleRange(0.2,0.66);
      Pushrod.scaleRange(0.19,0.25);
      Grabber.setPosition(1);
      Pushrod.setPosition(0);
      while (opModeIsActive()) {                                                                    //Run phase
        prevgrab=grab;
        prevpush=push;
        prevfly=flywheel;
        prevcoll=collector;
        double for_axis = LogarithmifyInput(gamepad1.left_stick_y,LogPower);
        double strafe_axis = LogarithmifyInput(gamepad1.left_stick_x,LogPower);
        double turn_axis = LogarithmifyInput(gamepad1.right_stick_x,LogPower);
        double worm_axis = LogarithmifyInput(gamepad2.left_stick_y,LogPower);
        grab = gamepad2.left_bumper;
        flywheel = gamepad2.right_bumper;
        push =((int)(gamepad2.right_trigger+0.25) != 0);
        collector =((int)(gamepad2.left_trigger+0.25) != 0);
        FRmotor.setPower(for_axis + strafe_axis + turn_axis);                                       //wheel motor movement
        RRmotor.setPower(for_axis - strafe_axis + turn_axis);
        FLmotor.setPower(-for_axis + strafe_axis + turn_axis);
        RLmotor.setPower(-for_axis - strafe_axis + turn_axis);
        Worm.setPower(-worm_axis);
        if(!prevgrab && grab) {                                                                     //arm servo movement
          Grabber.setPosition(1-Grabber.getPosition());                                          //0.88=0.66(open state)+0.22(closed state)
        }
        if(!prevpush && push){
          Pushrod.setPosition(1);
          sleep(100);
          Pushrod.setPosition(0);
        }
        if(!prevfly&&flywheel){
          Flywheel.setPower(1-Flywheel.getPower());
        }
        if(!prevcoll&&collector){
          Collector.setPower(1-Collector.getPower());
        }
      }
    }
  }

  public static double LogarithmifyInput(double input,int power) {
    return Math.abs(Math.pow(input,power))*Math.signum(input);
  }
}

