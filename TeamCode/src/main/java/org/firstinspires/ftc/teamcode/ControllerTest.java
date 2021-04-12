package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "Bingus Controller Test")
public class ControllerTest extends CommonOpMode {
  @Override
  public void runOpMode() {
    Initialize(hardwareMap,false);
    waitForStart();
    if (opModeIsActive()) {
      Grabber.setPosition(1);
      Pushrod.setPosition(0);
      while (opModeIsActive()) {
        composeInputs();
        operatePeripherals();
        double vel = FlywheelEx.getVelocity()/28*60;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Flywheel RPM: ",vel);
        telemetry.addData("Flywheel Pos: ", Flywheel.getCurrentPosition());
        telemetry.addData("Heading in degrees:",angles.firstAngle);
        telemetry.addData("Sensor Calibration Status:",imu.getCalibrationStatus());
        telemetry.addData("Sensor Mode:",imu.getParameters().mode);
        telemetry.addData("Encoder Data:",FRmotor.getCurrentPosition());
        telemetry.update();
      }
    }
  }
}