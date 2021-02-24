package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@TeleOp(name = "Calibrate Encoder")
public class EncoderCalibration extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotorEx Worm = (DcMotorEx) hardwareMap.get(DcMotor.class, "worm");
        waitForStart();
        if(opModeIsActive()){
            Worm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Worm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while(opModeIsActive()){
                double worm_axis = Math.pow(gamepad2.right_stick_y,3.0);
                Worm.setPower(worm_axis);
                telemetry.addData("Current Worm Position",Worm.getCurrentPosition());
                telemetry.addData("Target Worm Position",Worm.getTargetPosition());
                telemetry.addData("Worm Speed",Worm.getVelocity());
                telemetry.update();
            }
        }
    }
}
