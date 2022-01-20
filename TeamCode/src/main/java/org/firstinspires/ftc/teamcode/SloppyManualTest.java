package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="1000-7?")
public class SloppyManualTest extends CommonOpMode {
    @Override
    public void runOpMode(){
        Initialize(hardwareMap,false, BingusPipeline.StartLine.RIGHT);
        for (DcMotorEx motor:movementMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        waitForStart();
        freightServo.setPosition(1);
        while (opModeIsActive()){
            forward_axis = -logifyInput(gamepad1.left_stick_y, 2);
            strafe_axis = -logifyInput(gamepad1.left_stick_x,2);
            turn_axis = -logifyInput(gamepad1.right_stick_x,2);
            collector = (gamepad2.dpad_down || gamepad2.dpad_up);
            freight = gamepad2.right_bumper;
            switch ((int) freightServo.getPosition()){
                case 1:
                    lowerArmLimit = 20;
                    break;
                case 0:
                    lowerArmLimit = 300;
                    break;
            }
            if(-riserMotor.getCurrentPosition()<lowerArmLimit){
                riser_axis=(-gamepad2.right_stick_y+Math.abs(gamepad2.right_stick_y))/2;
                restrictor = 1;
            }
            else if(-riserMotor.getCurrentPosition()>upperArmLimit){
                riser_axis=(-gamepad2.right_stick_y-Math.abs(gamepad2.right_stick_y))/2;
                restrictor = 0.33;
            }
            else {
                riser_axis = -gamepad2.right_stick_y;
                restrictor = 0.33;
            }

            if(!previous_collector&&collector){
                if(gamepad2.dpad_down){
                    collectorMotor.setPower((-1-Math.abs(collectorMotor.getPower())*Math.signum(Math.signum(collectorMotor.getPower())-1)));
                }
                else collectorMotor.setPower((1-collectorMotor.getPower()*Math.signum(Math.signum(collectorMotor.getPower())+1)));
            }

            if(!previous_freight && freight && -riserMotor.getCurrentPosition()>300){
                freightServo.setPosition(1-freightServo.getPosition());
            }

            movementMotors[0].setPower(Range.clip(forward_axis - strafe_axis - turn_axis,-1,1)*restrictor);
            movementMotors[1].setPower(Range.clip(forward_axis + strafe_axis - turn_axis,-1,1)*restrictor);
            movementMotors[2].setPower(Range.clip(forward_axis - strafe_axis + turn_axis,-1,1)*restrictor);
            movementMotors[3].setPower(Range.clip(forward_axis + strafe_axis + turn_axis,-1,1)*restrictor);
            for (int i=0;i<4;i++){
                telemetry.addData(String.format("Pos%s:",i), movementMotors[i].getCurrentPosition());
            }
            previous_freight = freight;
            previous_collector = collector;
            telemetry.addData("Riser: ",riserMotor.getCurrentPosition());
            switch ((int) restrictor){
                case 1:
                    telemetry.addData("Speed: ", "HIGH - ARM DISENGAGED, FULL SPEED");
                    break;
                default:
                    telemetry.addData("Speed: ", "LOW - ARM ENGAGED");
            }
            telemetry.update();
        }
    }
}
