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
            idle();
        }
        if(opModeIsActive()){
            ReadyPeripherals();
            while(opModeIsActive()){
                if(!ExecuteFlag) {
                    AutoRingLaunch();
                    switch (ringData){
                        case ZERO:
                            OrientToDegrees(90);
                            MoveWithEncoder(200, 2);
                            DeployArm();
                            MoveWithEncoder(200, 0);
                            RetractArm();
                            OrientToDegrees(0);
                            break;
                        case ONE:
                            MoveWithEncoder(285, 2);
                            OrientToDegrees(-90);
                            DeployArm();
                            MoveWithEncoder(285, 0);
                            RetractArm();
                            OrientToDegrees(0);
                            MoveWithEncoder(285, 0);
                            break;
                        case FOUR:
                            MoveWithEncoder(866, 2);
                            OrientToDegrees(90);
                            MoveWithEncoder(200, 2);
                            DeployArm();
                            MoveWithEncoder(200, 0);
                            RetractArm();
                            OrientToDegrees(0);
                            MoveWithEncoder(866, 0);
                            break;
                    }
                    ExecuteFlag=true;
                }
                else idle();
            }
        }
    }
}