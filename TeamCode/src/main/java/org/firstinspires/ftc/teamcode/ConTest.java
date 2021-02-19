package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Controller Test: Commit Of The Year Edition", group = "")
public class ConTest extends LinearOpMode {
  @Override
  public void runOpMode() {                                                                         //Initialization phase
    DcMotor FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");                        //Hardware mapping and declaration of devices
    DcMotor RRmotor = hardwareMap.get(DcMotor.class, "RRmotor");
    DcMotor FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
    DcMotor RLmotor = hardwareMap.get(DcMotor.class, "RLmotor");
    DcMotor Worm = hardwareMap.get(DcMotor.class, "worm");
    Servo Grabber = hardwareMap.get(Servo.class,"grabber");
    waitForStart();
    if (opModeIsActive()) {                                                                         //Pre-run phase
      boolean grab = false;
      while (opModeIsActive()) {                                                                    //Run phase
        boolean prevgrab = grab;                                                                    //Data fetching
        double for_axis = LogarithmifyInput(gamepad1.left_stick_y,2);
        double strafe_axis = LogarithmifyInput(gamepad1.left_stick_x,2);
        double turn_axis = LogarithmifyInput(gamepad1.right_stick_x,2);
        double worm_axis = LogarithmifyInput(gamepad2.right_stick_y,2);
        grab = gamepad2.right_bumper;
        FRmotor.setPower(for_axis + strafe_axis + turn_axis);                                       //wheel motor movement
        RRmotor.setPower(for_axis - strafe_axis + turn_axis);
        FLmotor.setPower(-for_axis + strafe_axis + turn_axis);
        RLmotor.setPower(-for_axis - strafe_axis + turn_axis);
        Worm.setPower(worm_axis);
        if(!prevgrab && grab) {                                                                     //arm servo movement
          Grabber.setPosition(0.88-Grabber.getPosition());                                          //0.88=0.66(open state)+0.22(closed state)
        }
      }
    }
  }

  public static double LogarithmifyInput(double input,int power) {
    return Math.pow(input,power)*Math.signum(input);
  }
}

