package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous RED edition: Left Edition",preselectTeleOp = "Bingus Controller Test")
public class AutoRedLeft extends CommonOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode(){
        Initialize(hardwareMap,true,BingusPipeline.StartLine.LEFT);
        color = Color.RED;
        while((!isStarted())&&(!isStopRequested())){
            ringData = pipeline.ComposeTelemetry(telemetry);
            idle();
        }
        if(opModeIsActive()){
            ReadyPeripherals();
            while(opModeIsActive()){
                if(!ExecuteFlag) {
                    AutoRingLaunch();
                    switch (ringData){
                        case ZERO:
                            MoveWithEncoder(600, 2);
                            OrientToDegrees(90);
                            MoveWithEncoder(75, 0);
                            DeployArm();
                            MoveWithEncoder(800, 2);
                            RetractArm();
                            OrientToDegrees(0);
                            MoveWithEncoder(100, 0);
                            OrientToDegrees(0);
                            break;
                        case ONE:
                            MoveWithEncoder(1200, 2);
                            OrientToDegrees(90);
                            DeployArm();
                            MoveWithEncoder(400, 2);
                            RetractArm();
                            OrientToDegrees(0);
                            MoveWithEncoder(250, 0);
                            OrientToDegrees(0);
                            break;
                        case FOUR:
                            MoveWithEncoder(650, 2);
                            OrientToDegrees(100);
                            OrientToDegrees(145);
                            MoveWithEncoder(225, 0);
                            DeployArm();
                            MoveWithEncoder(800, 2);
                            RetractArm();
                            OrientToDegrees(0);
                            MoveWithEncoder(75, 0);
                            OrientToDegrees(0);
                            break;
                    }
                    ExecuteFlag=true;
                }
                else idle();
            }
        }
    }
}