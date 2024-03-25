package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(preselectTeleOp = "Toyota Mark II Simulation")
public class AutoTime extends CommonOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Initialize(hardwareMap);
        riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Action turn = drive.actionBuilder(new Pose2d(0,0,0.5*Math.PI))
                .turnTo(0)
                .build();
        double what = 12/drive.voltageSensor.getVoltage();
        while(opModeInInit()){

        }
        riserMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        riserMotor.setPower(-1);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.5*what, -what), 0)); //TODO
        safeSleep(1000);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
        safeSleep(800);
        riserMotor.setPower(0);
        riserServoA.setPosition(1);
        riserServoB.setPosition(1);
        //Actions.runBlocking(turn);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.5*what, 0),0));
        safeSleep(1000);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
        safeSleep(100);
        pusherServo.setPosition(1);
        safeSleep(500);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-0.5*what, 0),0));
        safeSleep(100);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.1*what, what),0));
        safeSleep(600);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
        while(opModeIsActive() && !isStopRequested()){
            idle();
        }
    }
}
