package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;
@TeleOp(name="Movement Test")
public class MovementTest extends CommonOpMode {
    BNO055IMU imu;
    Orientation angles = new Orientation();
    Acceleration gravity = new Acceleration();
    double for_axis;
    double strafe_axis;
    double turn_axis;
    double worm_axis;
    int LogPower = 3;
    @Override
    public void runOpMode(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "calib.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        waitForStart();
        if(opModeIsActive()){
            FRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while(opModeIsActive()){
                for_axis = LogarithmifyInput(gamepad1.left_stick_y,LogPower);
                strafe_axis = LogarithmifyInput(gamepad1.left_stick_x,LogPower);
                turn_axis = LogarithmifyInput(gamepad1.right_stick_x,LogPower);
                worm_axis = LogarithmifyInput(gamepad2.left_stick_y,LogPower);
                FRmotor.setPower(for_axis + strafe_axis + turn_axis);                                       //wheel motor movement
                RRmotor.setPower(for_axis - strafe_axis + turn_axis);
                FLmotor.setPower(for_axis - strafe_axis - turn_axis);
                RLmotor.setPower(for_axis + strafe_axis - turn_axis);
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity  = imu.getGravity();
                telemetry.addData("status", imu.getSystemStatus().toShortString());
                telemetry.addData("calib", imu.getCalibrationStatus().toString());
                telemetry.addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));
                telemetry.addData("roll", formatAngle(angles.angleUnit, angles.secondAngle));
                telemetry.addData("pitch", formatAngle(angles.angleUnit, angles.thirdAngle));
                telemetry.addData("grvty", gravity.toString());
                telemetry.addData("mag", String.format(Locale.getDefault(), "%.3f",
                        Math.sqrt(gravity.xAccel*gravity.xAccel
                                + gravity.yAccel*gravity.yAccel
                                + gravity.zAccel*gravity.zAccel)));
                telemetry.update();
            }
        }
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public static double LogarithmifyInput(double input,int power) {
        return Math.abs(Math.pow(input,power))*Math.signum(input);
    }
}
