package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Toyota Mark II Simulation")
public class GigachadTeleOp extends TeleOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        boolean direct = false;
        Initialize(hardwareMap);
        riserServoA.setPosition(1);
        riserServoB.setPosition(-1);
        pusherServo.setPosition(0);
        riserMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //collectorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //drive.update();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.y && !direct) {
                direct = true;
            }
            if (gamepad2.x && direct) {
                direct = false;
            }
            // INPUT GATHERING
            forward_axis = logifyInput(gamepad1.left_stick_y, 2.718);
            strafe_axis = logifyInput(gamepad1.left_stick_x, 2.718);
            turn_axis = 0.75 * logifyInput(gamepad1.right_stick_x, 2.718);
            riserArm = gamepad2.right_trigger > 0.1;
            pusher = gamepad2.right_bumper;
            riser_axis = (gamepad2.right_stick_y > 0 ? 1 : 1) * gamepad2.right_stick_y;
            riserPos = -riserMotor.getCurrentPosition();
            collector_motors = logifyInput(gamepad2.left_stick_y, 1.5);
            ;

            // RISER SAFETY
            if (!direct) {
                if (armLimiter.isPressed()) {
                    riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    riserMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                if (riserPos <= 1) {
                    riser_axis = Math.min(0, riser_axis);
                }
                if (riserPos<armExtensionToEncoderTicks(-400)){
                    restrictor = restrictorCap;
                } else restrictor = 0.33;
                if (riserPos > upperArmLimit) {
                    riser_axis = Math.max(0, riser_axis);
                }
            } else restrictor = restrictorCap;


            /* POWER APPLICATION: most likely already working as intended, do not touch! */

            // drive power
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(
                    -forward_axis * restrictor,
                    -strafe_axis * restrictor),
                    -turn_axis * restrictor
            ));
            riserMotor.setPower(riser_axis);
            collectorMotor.setPower(collector_motors);

            //freight holder position
            if (!pRiserArm && riserArm) {
                riserServoA.setPosition(1 - riserServoA.getPosition());
                riserServoB.setPosition(1 - riserServoB.getPosition());
            }
            pusherServo.setPosition(pusher ? 0.05 : 0);
            pRiserArm = riserArm;

            // TELEMETRY
            /*
            telemetry.addData("Speed: ", restrictor==restrictorCap?"HIGH":"LOW");
            telemetry.addData("Riser position: ", (int)(riserPos/2880*975));
            telemetry.addData("Color: ","%d %d",lineSensor.blue()-lineSensor.green(), lineSensor.red()-lineSensor.green());
            telemetry.addData("Button: ",armLimiter.isPressed());
            telemetry.addData("Direct enabled: ", direct?"YES (x to disable)":"no (y to enable)");
            telemetry.update();
            //drive.update();

             */
        }
    }
}
