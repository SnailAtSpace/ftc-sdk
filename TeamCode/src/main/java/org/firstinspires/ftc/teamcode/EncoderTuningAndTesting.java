package org.firstinspires.ftc.teamcode;

import android.graphics.Interpolator;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.sun.tools.javac.util.ArrayUtils;
import com.sun.tools.javac.util.List;

import java.util.Arrays;
import java.util.Collections;

@TeleOp
public class EncoderTuningAndTesting extends CommonOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Initialize(hardwareMap,false, BingusPipeline.StartLine.RIGHT);
        int[] motorsPos = new int[4];
        riserMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        riserMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while(opModeIsActive()){
            for (int i=0;i<4;i++){
                movementMotors[i].setPower(gamepad1.left_stick_y);
                movementMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorsPos[i]=movementMotors[i].getCurrentPosition();
                telemetry.addData(String.format("Pos%s:",i), motorsPos);
            }
            riserMotor.setPower(gamepad2.left_stick_y*(0.25));
            Arrays.sort(motorsPos);
            telemetry.addData("Arm: ", riserMotor.getCurrentPosition());
            telemetry.addData("Deviation: ", motorsPos[3]-motorsPos[0]);
            telemetry.update();
        }
    }
}
