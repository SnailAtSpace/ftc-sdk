package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="1000-7?")
public class SloppyManualTest extends CommonOpMode {
    @Override
    public void runOpMode(){
        Initialize(hardwareMap,false);
        riserMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        for (DcMotorEx motor:movementMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        waitForStart();
        freightServo.setPosition(1);
        while (opModeIsActive()){
            forward_axis = gamepad1.left_stick_y;
            strafe_axis = gamepad1.left_stick_x;
            turn_axis = -0.65*gamepad1.right_stick_x;
            collector = (gamepad2.dpad_down || gamepad2.dpad_up);
            freight = gamepad2.right_bumper;
            carousel_axis = gamepad2.left_stick_x;
            double riserPos = riserMotor.getCurrentPosition();
            switch ((int) Math.round(freightServo.getPosition())){
                case 1:
                    lowerArmLimit = 5;
                    break;
                case 0:
                    lowerArmLimit = safeArmLimit;
                    break;
            }
            if(riserPos<lowerArmLimit){
                riser_axis=(-gamepad2.right_stick_y+Math.abs(gamepad2.right_stick_y))/2;//cap lower-bound
            }
            else if(riserPos>upperArmLimit){
                riser_axis=(-gamepad2.right_stick_y-Math.abs(gamepad2.right_stick_y))/2;
                restrictor = 0.33;
            }
            else {
                restrictor = 0.33;
                riser_axis = -gamepad2.right_stick_y;
            }
            if(riserPos<=10){
                restrictor = restrictorCap;
            }

            if(!previous_collector&&collector){
                if(gamepad2.dpad_down){
                    collectorMotor.setPower((-maxCollPower-Math.abs(collectorMotor.getPower())*Math.signum(Math.signum(collectorMotor.getPower())-1)));
                }
                else collectorMotor.setPower((maxCollPower-Math.abs(collectorMotor.getPower())*Math.signum(Math.signum(collectorMotor.getPower())+1)));
            }

            if(!previous_freight && freight && riserPos>safeArmLimit){
                freightServo.setPosition(1-freightServo.getPosition());
            }
            carouselMotor.setPower(carousel_axis);
            movementMotors[0].setPower(Range.clip(forward_axis - strafe_axis + turn_axis,-1,1)*restrictor);
            movementMotors[1].setPower(Range.clip(forward_axis + strafe_axis + turn_axis,-1,1)*restrictor);
            movementMotors[2].setPower(Range.clip(forward_axis - strafe_axis - turn_axis,-1,1)*restrictor);
            movementMotors[3].setPower(Range.clip(forward_axis + strafe_axis - turn_axis,-1,1)*restrictor);
            riserMotor.setPower(riser_axis*0.66);
            for (int i=0;i<4;i++){
                telemetry.addData(String.format("Pos%s:",i), movementMotors[i].getCurrentPosition());
            }
            previous_freight = freight;
            previous_collector = collector;
            telemetry.addData("Riser: ",riserPos + " " + riser_axis + " " + lowerArmLimit);
            switch ((int)Math.round(restrictor)){
                case 1:
                    telemetry.addData("Speed: ", "HIGH - ARM DISENGAGED, FULL SPEED");
                    break;
                default:
                    telemetry.addData("Speed: ", "LOW - ARM ENGAGED");
            }
            telemetry.addData("Servo: ", freightServo.getPosition() + " "+ (int) Math.round(freightServo.getPosition()));
            telemetry.addData("Stick: ", gamepad2.right_stick_y+" "+gamepad2.right_stick_y);
            telemetry.update();
        }
    }
}
