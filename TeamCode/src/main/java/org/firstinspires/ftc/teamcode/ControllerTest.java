package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.stream.Collector;

//test commit 2
@TeleOp(name = "Bingus Controller Test")
public class ControllerTest extends LinearOpMode {
  CommonValues commonValues = new CommonValues();
  @Override
  public void runOpMode() {
    final int LogPower=3;
    boolean grab = false;
    boolean push = false;
    boolean collector = false;
    boolean flywheel = false;
    boolean prevgrab;
    boolean prevpush;
    boolean prevfly;
    boolean prevcoll;
    double for_axis;
    double strafe_axis;
    double turn_axis;
    double worm_axis;
    boolean isFlywheelRunning = false;
    commonValues.Initialize(hardwareMap);
    waitForStart();
    if (opModeIsActive()) {                                                                         //Pre-run phase
      commonValues.Grabber.scaleRange(0.2,0.66);
      commonValues.Pushrod.scaleRange(0.19,0.3);
      commonValues.Grabber.setPosition(1);
      commonValues.Pushrod.setPosition(0);
      DcMotorEx FlywheelEx = (DcMotorEx)(commonValues.Flywheel);
      //FlywheelEx.setVelocityPIDFCoefficients();
      while (opModeIsActive()) {                                                                    //Run phase
        prevgrab=grab;
        prevpush=push;
        prevfly=flywheel;
        prevcoll=collector;
        for_axis = LogarithmifyInput(gamepad1.left_stick_y,LogPower);
        strafe_axis = LogarithmifyInput(gamepad1.left_stick_x,LogPower);
        turn_axis = LogarithmifyInput(gamepad1.right_stick_x,LogPower);
        worm_axis = LogarithmifyInput(gamepad2.left_stick_y,LogPower);
        grab = gamepad2.left_bumper;
        flywheel = gamepad2.right_bumper;
        push =((int)(gamepad2.right_trigger+0.25) != 0);
        collector = (gamepad2.dpad_down)||(gamepad2.dpad_up);
        commonValues.FRmotor.setPower(for_axis + strafe_axis + turn_axis);                                       //wheel motor movement
        commonValues.RRmotor.setPower(for_axis - strafe_axis + turn_axis);
        commonValues.FLmotor.setPower(-for_axis + strafe_axis + turn_axis);
        commonValues.RLmotor.setPower(-for_axis - strafe_axis + turn_axis);
        commonValues.Worm.setPower(worm_axis);
        if(!prevgrab && grab) {                                                                     //arm servo movement
          commonValues.Grabber.setPosition(1-commonValues.Grabber.getPosition());                                          //0.88=0.66(open state)+0.22(closed state)
        }
        if(!prevpush && push){
          commonValues.Pushrod.setPosition(1);
          sleep(100);
          commonValues.Pushrod.setPosition(0);
        }
        if(!prevfly&&flywheel){
          FlywheelEx.setVelocity(4500-(isFlywheelRunning?1:0)*4500);
          isFlywheelRunning=!isFlywheelRunning;
        }
        if(!prevcoll&&collector){
          if(gamepad2.dpad_down){
            commonValues.Collector.setPower(-0.75-Math.abs(commonValues.Collector.getPower())*Math.signum(Math.signum(commonValues.Collector.getPower())-1));
          }
          else commonValues.Collector.setPower(0.75-commonValues.Collector.getPower()*Math.signum(Math.signum(commonValues.Collector.getPower())+1));
        }
        double vel = FlywheelEx.getVelocity()/28*60;
        telemetry.addData("Flywheel Velocity: ",vel);
        telemetry.update();
      }
    }
  }

  public static double LogarithmifyInput(double input,int power) {
    return Math.abs(Math.pow(input,power))*Math.signum(input);
  }
}

