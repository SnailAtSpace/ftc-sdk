package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class EncoderCal extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotor Worm;
        Worm = hardwareMap.get(DcMotor.class, "Wmotor");
        waitForStart();
        if(opModeIsActive()){
            Worm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Worm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while(opModeIsActive()){
                Worm.setPower(gamepad1.left_stick_y);
                telemetry.addData("Encoder Data:",Worm.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}
