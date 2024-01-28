package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Toyota Supra Simulation")
public class DristMode extends CommonOpMode{
    double brake_axis;
    boolean reverse = false, handbrake = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Initialize(hardwareMap);
        waitForStart();
        while(opModeIsActive()&&!isStopRequested()){
            //GAZU GAZU GAZU
            double speed = drive.updatePoseEstimateAndGetActualVel().transVel.x;
            forward_axis = gamepad1.right_trigger;
            brake_axis = gamepad1.left_trigger;
            turn_axis = gamepad1.right_stick_x*Math.signum(speed);
            handbrake = gamepad1.left_bumper;
            if(gamepad1.dpad_down)reverse=true;
            if(gamepad1.dpad_up)reverse=false;
            Pose2d pose = drive.pose;
            double forward = Math.max(forward_axis-brake_axis*8,-0.05)*(reverse?-1:1);
            drive.setMotorPowers(
                    handbrake?(-0.1*Math.signum(speed)):forward+turn_axis*speed*drive.inPerTick/80
                    ,forward
                    ,forward
                    ,handbrake?(-0.1*Math.signum(speed)):forward-turn_axis*speed*drive.inPerTick/80
            );
        }
    }
}
