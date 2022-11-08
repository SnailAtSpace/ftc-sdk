package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

@TeleOp(name="W+M1")
public class GigachadTeleOp extends CommonOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode(){
        Initialize(hardwareMap,false);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        riserServo.setPosition(0);
        drive.update();
        waitForStart();
        while (opModeIsActive()){

            // LL VALUE ASSIGNMENT

            //INPUT GATHERING
            forward_axis = gamepad1.left_stick_y;
            strafe_axis = gamepad1.left_stick_x;
            turn_axis = 0.65*gamepad1.right_stick_x;
            riserArm = gamepad2.right_bumper;
            riser_axis = gamepad2.right_stick_y;
            riserPos = riserMotor.getCurrentPosition();

            //RISER SAFETY
//            if(riserPos<10){
//                restrictor = restrictorCap;
//                riser_axis = Math.max(0,riser_axis);
//            }
//            else if(riserPos>upperArmLimit){
//                riser_axis = Math.min(0,riser_axis);
//            }
//            else{
//                restrictor = 0.33;
//            }


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
                riserServo.setPosition(0.2-riserServo.getPosition());
            }
            pRiserArm = riserArm;

            //TELEMETRY
            telemetry.addData("Drive ticks: ", String.format("%1$d %2$d %3$d %4$d", drive.getWheelTicks().toArray()));
            telemetry.addData("RR Position: ", drive.getPoseEstimate().toString());
            if(restrictor == restrictorCap){
                telemetry.addData("Speed: ", "HIGH - ARM DISENGAGED, FULL SPEED");
            } else {
                telemetry.addData("Speed: ", "LOW - ARM ENGAGED, AVOID RAPID MOVEMENTS");
            }
            telemetry.addData("Riser position: ", riserPos);
            telemetry.addData("Dead wheels: ", String.format("%.3f %.3f %.3f", ((StandardTrackingWheelLocalizer)drive.getLocalizer()).getWheelVelocities().toArray()));
            telemetry.update();
            drive.update();
        }
    }
}
