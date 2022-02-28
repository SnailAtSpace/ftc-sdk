package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
            riser_axis = -gamepad2.right_stick_y;
            double riserPos = riserMotor.getCurrentPosition();

            switch ((int) Math.round(freightServo.getPosition())){
                case 1:
                    lowerArmLimit = 5;
                    break;
                case 0:
                    lowerArmLimit = safeArmLimit;
                    break;
            }

            if(armButton.isPressed()){
                restrictor = 0.33;
                previousButtonState = true;
                riser_axis = Math.max(0,riser_axis);
                freight = false;
                riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                riserMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            else{
                restrictor = restrictorCap;
            }

            if(riserPos>upperArmLimit){
                riser_axis = Math.min(0,riser_axis);
            }

            if(!previousCollector&&collector){
                if(gamepad2.dpad_down){
                    collectorMotor.setPower((-maxCollPower-Math.abs(collectorMotor.getPower())*Math.signum(Math.signum(collectorMotor.getPower())-1)));
                }
                else collectorMotor.setPower((maxCollPower-Math.abs(collectorMotor.getPower())*Math.signum(Math.signum(collectorMotor.getPower())+1)));
            }

            if(!previousFreight && freight && riserPos>safeArmLimit){
                freightServo.setPosition(1-freightServo.getPosition());
            }
            carouselMotor.setPower(carousel_axis);
            movementMotors[0].setPower(Range.clip(forward_axis - strafe_axis + turn_axis,-1,1)*restrictor);
            movementMotors[1].setPower(Range.clip(forward_axis + strafe_axis + turn_axis,-1,1)*restrictor);
            movementMotors[2].setPower(Range.clip(forward_axis - strafe_axis - turn_axis,-1,1)*restrictor);
            movementMotors[3].setPower(Range.clip(forward_axis + strafe_axis - turn_axis,-1,1)*restrictor);
            riserMotor.setPower(riser_axis);
            for (int i=0;i<4;i++){
                telemetry.addData(String.format("Pos%s:",i), movementMotors[i].getCurrentPosition());
            }
            previousFreight = freight;
            previousCollector = collector;
            switch ((int)Math.round(restrictor)){
                case 1:
                    telemetry.addData("Speed: ", "HIGH - ARM DISENGAGED, FULL SPEED");
                    break;
                default:
                    telemetry.addData("Speed: ", "LOW - ARM ENGAGED");
            }
            telemetry.addData("Button state: ",armButton.isPressed());
            telemetry.addData("Element: ", freightSensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
