package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ConTest (Blocks to Java)", group = "")
public class ConTest extends LinearOpMode {

  private DcMotor FRmotor;
  private DcMotor RRmotor;
  private DcMotor FLmotor;
  private DcMotor RLmotor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    boolean controlMode;
    double for_axis;
    double strafe_axis;
    double turn_axis=0;
    FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");
    RRmotor = hardwareMap.get(DcMotor.class, "RRmotor");
    FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
    RLmotor = hardwareMap.get(DcMotor.class, "RLmotor");

    // Reverse one of the drive motors.
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    waitForStart();
    if (opModeIsActive()) {
      controlMode = false;
      // Put run blocks here.
      while (opModeIsActive()) {
          for_axis = gamepad1.left_stick_y;
          strafe_axis = gamepad1.left_stick_x;
          turn_axis = gamepad1.right_stick_x;
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        FRmotor.setPower(for_axis+strafe_axis*2.0+turn_axis*2.0);
        RRmotor.setPower(for_axis-strafe_axis*2.0+turn_axis*2.0);
        FLmotor.setPower(-for_axis+strafe_axis*2.0+turn_axis*2.0);
        RLmotor.setPower(-for_axis-strafe_axis*2.0+turn_axis*2.0);
      }
    }
  }

  /**
   * Describe this function...
   */
  private double convert(boolean bool) {
    double ret;

    if (bool) {
      ret = 1;
    } else {
      ret = 0;
    }
    return ret;
  }
}
