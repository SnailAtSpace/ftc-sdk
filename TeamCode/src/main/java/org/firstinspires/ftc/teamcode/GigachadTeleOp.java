package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Toyota Mark II Simulation")
public class GigachadTeleOp extends TeleOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode(){
        Initialize(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        riserServo.setPosition(0);
        riserMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.update();
        waitForStart();
        while (opModeIsActive()){

            // INPUT GATHERING
            forward_axis = logifyInput(gamepad1.left_stick_y,2.718);
            strafe_axis = logifyInput(gamepad1.left_stick_x,2.718);
            turn_axis = 0.75*logifyInput(gamepad1.right_stick_x,2.718);
            riserArm = gamepad2.right_bumper;
            riser_axis = (gamepad2.right_stick_y>0?0.8:1)*gamepad2.right_stick_y;
            riserPos = -riserMotor.getCurrentPosition();

            // RISER SAFETY
            if(armLimiter.isPressed()){
                riser_axis = Math.min(0,riser_axis);
                riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                riserMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (riserPos<armExtensionToEncoderTicks(-400)){
                restrictor = restrictorCap;
            }
            else restrictor = 0.33;
            if(riserPos>upperArmLimit){
                riser_axis = Math.max(0,riser_axis);
            }

            /* POWER APPLICATION: most likely already working as intended, do not touch! */

            //drive power
            drive.setWeightedDrivePower(new Pose2d(
                    -forward_axis*restrictor,
                    -strafe_axis*restrictor,
                    -turn_axis*restrictor
            ));
            riserMotor.setPower(riser_axis);

            //freight holder position
            if(!pRiserArm && riserArm){
                riserServo.setPosition(1-riserServo.getPosition());
            }
            pRiserArm = riserArm;

            // TELEMETRY
            telemetry.addData("Speed: ", restrictor==restrictorCap?"HIGH":"LOW");
            telemetry.addData("Riser position: ", (int)(riserPos/2880*975));
            telemetry.addData("Color: ", lineSensor.blue()-lineSensor.green());
            telemetry.addData("Button: ",armLimiter.isPressed());
            telemetry.update();
            drive.update();
        }
    }
}
