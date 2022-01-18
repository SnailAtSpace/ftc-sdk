package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.stream.Collector;

@TeleOp(name="1000-7?")
public class SloppyManualTest extends CommonOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Initialize(hardwareMap,false, BingusPipeline.StartLine.RIGHT);
        waitForStart();
        while (opModeIsActive()){
            forward_axis = logifyInput(gamepad1.left_stick_y, 2);
            strafe_axis = logifyInput(gamepad1.left_stick_x,2);
            turn_axis = logifyInput(gamepad1.right_stick_x,2);
            riser_axis = logifyInput(gamepad2.right_stick_y,2);
            collector = (gamepad2.dpad_down || gamepad2.dpad_up);
                        for (DcMotorEx motor:movementMotors) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            riserMotor.setPower(riser_axis);
            if(!previous_collector&&collector){
                if(gamepad2.dpad_down){
                    collectorMotor.setPower(-1-Math.abs(collectorMotor.getPower())*Math.signum(Math.signum(collectorMotor.getPower())-1));
                }
                else collectorMotor.setPower(1-collectorMotor.getPower()*Math.signum(Math.signum(collectorMotor.getPower())+1));
            }
            movementMotors[0].setPower(Range.clip(forward_axis - strafe_axis - turn_axis,-1,1)*restrictor);
            movementMotors[1].setPower(Range.clip(forward_axis + strafe_axis + turn_axis,-1,1)*restrictor);
            movementMotors[2].setPower(Range.clip(forward_axis + strafe_axis - turn_axis,-1,1)*restrictor);
            movementMotors[3].setPower(Range.clip(forward_axis - strafe_axis + turn_axis,-1,1)*restrictor);
            freightServo.setPosition((gamepad2.left_stick_y+1)/2.0);
            telemetry.addData("Pos of arm: ",movementMotors[0].getCurrentPosition());
            telemetry.update();
        }
    }
}
