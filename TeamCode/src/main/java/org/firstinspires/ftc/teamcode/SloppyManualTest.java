package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.io.File;
import java.util.Arrays;

@TeleOp(name="1000-7?")
public class SloppyManualTest extends CommonOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode(){
        Initialize(hardwareMap,false);
        riserMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        double[] drivePose;
        try {
            File file = AppUtil.getInstance().getSettingsFile("LastAutoPosition");
            drivePose = Arrays.stream(ReadWriteFile.readFile(file).split(" ")).mapToDouble(Double::parseDouble).toArray();
        }
        catch (Exception e){
            drivePose = new double[]{0,0,0};
        }
        waitForStart();
        drive.setPoseEstimate(new Pose2d(drivePose[0],drivePose[1],drivePose[2]));
        freightServo.setPosition(1);
        while (opModeIsActive()){
            //INPUT GATHERING
            forward_axis = gamepad1.left_stick_y;
            strafe_axis = gamepad1.left_stick_x;
            turn_axis = 0.65*gamepad1.right_stick_x;
            collector = (gamepad2.dpad_up?1:0)-(gamepad2.dpad_down?1:0);
            freight = gamepad2.right_bumper;
            carousel_axis = gamepad2.left_stick_x;
            riser_axis = -gamepad2.right_stick_y;
            double riserPos = riserMotor.getCurrentPosition();

            //RISER SAFETY
            if(armButton.isPressed()){
                restrictor = restrictorCap;
                riser_axis = Math.max(0,riser_axis);
                freight = false; //we can't deploy the freight holder when our arm is retracted
                riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                riserMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            else{
                if(riserPos>upperArmLimit){
                    riser_axis = Math.min(0,riser_axis);
                }
                restrictor = 0.33;
            }

            //COLLECTOR DOUBLE ELEMENT PREVENTION
            if(freightSensor.getDistance(DistanceUnit.MM)<40 && collector!=0){
                collector = -1;
            }

            /* POWER APPLICATION: most likely already working as intended, do not touch! */
            //collector power
            if(previousCollector!=collector && collector!=0){
                collectorMotor.setPower((collector*maxCollPower-Math.abs(collectorMotor.getPower())*Math.signum(Math.signum(collectorMotor.getPower())+collector)));
            }
            //freight holder position
            if(!previousFreight && freight){
                freightServo.setPosition(1-freightServo.getPosition());
            }
            //drive power
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -forward_axis*restrictor,
                            -strafe_axis*restrictor,
                            -turn_axis*restrictor
                    )
            );
            //miscellaneous power
            carouselMotor.setPower(carousel_axis);
            riserMotor.setPower(riser_axis);
            //utility variable assignment for buttons
            previousFreight = freight;
            previousCollector = collector;

            //TELEMETRY
            telemetry.addData(String.format("Drive ticks: "), String.format("%1$d %2$d %3$d %4$d", drive.getWheelTicks().toArray()));
            telemetry.addData("RR Position: ", drive.getPoseEstimate().toString());
            if(restrictor == restrictorCap){
                telemetry.addData("Speed: ", "HIGH - ARM DISENGAGED, FULL SPEED");
            } else {
                telemetry.addData("Speed: ", "LOW - ARM ENGAGED, AVOID RAPID MOVEMENTS");
            }
            telemetry.addData("Riser position: ", riserPos);
            telemetry.addData("Button state: ",armButton.isPressed());
            telemetry.addData("Element: ", freightSensor.getDistance(DistanceUnit.MM)<40?"YES":"NO");
            telemetry.update();
            drive.update();
        }
    }
}