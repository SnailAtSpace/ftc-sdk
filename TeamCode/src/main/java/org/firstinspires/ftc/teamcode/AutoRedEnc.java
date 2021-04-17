package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous RED edition: with BONUS encoders!",preselectTeleOp = "Bingus Controller Test MkI")
public class AutoRedEnc extends CommonOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode(){
        Initialize(hardwareMap,true);
        while((!isStarted())&&(!isStopRequested())){
            ringData = pipeline.ComposeTelemetry(telemetry);
            startLine = pipeline.getSide();
            idle();
        }
        if(opModeIsActive()){
            ReadyPeripherals();
            while(opModeIsActive()){
                if(!ExecuteFlag) {
                    AutoRingLaunch();
                    switch (ringData){
                        case ZERO:
                            MoveWithEncoder(300, 2);
                            OrientToDegrees(90);
                            DeployArm();
                            MoveWithEncoder(175, 2);
                            RetractArm();
                            OrientToDegrees(0);
                            break;
                        case ONE:
                            MoveWithEncoder(500, 2);
                            OrientToDegrees(-90);
                            DeployArm();
                            MoveWithEncoder(100, 2);
                            RetractArm();
                            OrientToDegrees(0);
                            MoveWithEncoder(100, 0);
                            break;
                        case FOUR:
                            MoveWithEncoder(1000, 2);
                            OrientToDegrees(90);
                            DeployArm();
                            MoveWithEncoder(175, 2);
                            RetractArm();
                            OrientToDegrees(0);
                            MoveWithEncoder(250, 0);
                            break;
                    }
                    ExecuteFlag=true;
                }
                else idle();
            }
        }
    }
}