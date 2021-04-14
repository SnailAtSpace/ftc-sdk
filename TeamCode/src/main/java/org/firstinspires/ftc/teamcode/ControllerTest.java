package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Bingus Controller Test MkI")
public class ControllerTest extends CommonOpMode {
    @Override
    public void runOpMode() {
        Initialize(hardwareMap, false);
        waitForStart();
        if (opModeIsActive()) {
            Grabber.setPosition(1);
            Pushrod.setPosition(0);
            while (opModeIsActive()) {
                composeInputs();
                operatePeripherals();
                double vel = FlywheelEx.getVelocity() / 28 * 60;
                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Flywheel RPM: ", vel);
                telemetry.addData("Heading in degrees:", angles.firstAngle);
                telemetry.addData("Flywheel PIDF: ",FlywheelEx.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
                telemetry.update();
            }
        }
    }
}

