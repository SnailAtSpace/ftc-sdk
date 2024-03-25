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
        double funny = 0.5;
        boolean direct = false;
        Initialize(hardwareMap);
        riserServoA.setPosition(0);
        riserServoB.setPosition(0);
        pusherServo.setPosition(0);
        launcherServo.setPosition(0);
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
            collector_axis = logifyInput(gamepad2.left_stick_y, 2.718);
            collector = (gamepad2.dpad_up?1:0)-(gamepad2.dpad_down?1:0);

            // RISER SAFETY
                if (armLimiter.isPressed()) {
                    riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    riserMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            if (!direct) {
                if (riserPos <= 20) {
                    riser_axis = Math.min(0, riser_axis);
                }
                if (riserPos<2000){
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
            collectorMotor.setPower(collector_axis>0?collector_axis*0.75:collector_axis*0.15);
            // OR
//            if(pCollector != collector && collector != 0){
//                collectorMotor.setPower((collector*0.5-Math.abs(collectorMotor.getPower())*Math.signum(Math.signum(collectorMotor.getPower())+collector)));
//            }

            //freight holder position
            if (!pRiserArm && riserArm) {
                riserServoA.setPosition(1 - riserServoA.getPosition());
                riserServoB.setPosition(1 - riserServoB.getPosition());
            }
            pusherServo.setPosition(pusher ? 0 : 1);
            pRiserArm = riserArm;
            launcherServo.setPosition(gamepad1.right_trigger*5);
            // TELEMETRY

            telemetry.addData("Speed: ", restrictor==restrictorCap?"HIGH":"LOW");
            telemetry.addData("Riser position: ", "%d %d", riserMotor.a.getCurrentPosition(), riserMotor.b.getCurrentPosition());
            telemetry.addData("Color: ",lineSensor.getLightDetected());
            telemetry.addData("Button: ",armLimiter.isPressed());
            telemetry.addData("Direct enabled: ", direct?"YES (x to disable)":"no (y to enable)");
            telemetry.addData("Wheels: ", "%d %d %d %d", drive.leftFront.getCurrentPosition()
                    , drive.leftBack.getCurrentPosition()
                    , drive.rightFront.getCurrentPosition()
                    , drive.rightBack.getCurrentPosition());
            telemetry.update();
            //drive.update();


        }
    }
}
