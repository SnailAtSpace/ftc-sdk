package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//test commit
@TeleOp(name = "Controller Test: Commit Of The Year Edition")
public class ConTest extends LinearOpMode {
  @Override
  public void runOpMode() {                                                                         //Initialization phase
    final int LogPower=3;
    DcMotor FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");                        //Hardware mapping and declaration of devices
    DcMotor RRmotor = hardwareMap.get(DcMotor.class, "RRmotor");
    DcMotor FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
    DcMotor RLmotor = hardwareMap.get(DcMotor.class, "RLmotor");
    DcMotor Flywheel = hardwareMap.get(DcMotor.class, "flywheel");
    DcMotor Worm = hardwareMap.get(DcMotor.class, "worm");
    Servo Grabber = hardwareMap.get(Servo.class,"grabber");
    Servo Pushrod = hardwareMap.get(Servo.class,"pushrod");
    FRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    FLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    waitForStart();
    if (opModeIsActive()) {                                                                         //Pre-run phase
      boolean grab = false;
      boolean flywheel=false;
      boolean push=false;
      Grabber.scaleRange(0.2,0.66);
      Grabber.setPosition(1);
      Flywheel.setPower(1);
      while (opModeIsActive()) {                                                                    //Run phase
        boolean prevgrab=grab;
        boolean prevpush=push;
        boolean prevfly=flywheel;
        double for_axis = LogarithmifyInput(gamepad1.left_stick_y,LogPower);
        double strafe_axis = LogarithmifyInput(gamepad1.left_stick_x,LogPower);
        double turn_axis = LogarithmifyInput(gamepad1.right_stick_x,LogPower);
        double worm_axis = LogarithmifyInput(gamepad2.right_stick_y,LogPower);
        grab = gamepad2.right_bumper;
        flywheel=gamepad2.left_bumper;
        int intpush=(int) (gamepad2.right_trigger+0.25);
        push = intpush!=0;
        FRmotor.setPower(for_axis + strafe_axis + turn_axis);                                       //wheel motor movement
        RRmotor.setPower(for_axis - strafe_axis + turn_axis);
        FLmotor.setPower(-for_axis + strafe_axis + turn_axis);
        RLmotor.setPower(-for_axis - strafe_axis + turn_axis);
        Worm.setPower(worm_axis);
        if(!prevgrab && grab) {                                                                     //arm servo movement
          Grabber.setPosition(1-Grabber.getPosition());                                          //0.88=0.66(open state)+0.22(closed state)
        }
        if(!prevpush&&push){
          Pushrod.setPosition(1);
          sleep(50);
          Pushrod.setPosition(0);
        }
        if(flywheel){
          Flywheel.setPower(1);
        }
      }
    }
  }

  public static double LogarithmifyInput(double input,int power) {
    return Math.pow(input,power)*Math.signum(input);
  }
}

