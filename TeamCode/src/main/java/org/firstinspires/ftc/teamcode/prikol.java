package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class prikol extends LinearOpMode {
    DcMotor RRmotor;
    @Override
    public void runOpMode(){
        RRmotor = hardwareMap.get(DcMotor.class, "RLmotor");
        RRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RRmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if(opModeIsActive()){
            RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RRmotor.setTargetPosition(100);
            RRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(opModeIsActive()){
                RRmotor.setPower(1);
                telemetry.addData("Pos: ",RRmotor.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}
