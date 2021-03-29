package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled
public class prikol extends LinearOpMode {
    DcMotor RRmotor;
    @Override
    public void runOpMode(){
        RRmotor = hardwareMap.get(DcMotor.class, "FWmotor");
        RRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RRmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if(opModeIsActive()){
            RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while(opModeIsActive()){
                RRmotor.setPower(1);
                telemetry.addData("Pos: ",((DcMotorEx)RRmotor).getVelocity()*360/7);
                telemetry.update();
            }
        }
    }
}
