package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous RED edition: with BONUS encoders!",preselectTeleOp = "Bingus Controller Test")
public class AutoRedEnc extends CommonOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode(){
        Initialize(hardwareMap,true);
        while((!isStarted())&&(!isStopRequested())){
            pipeline.ComposeTelemetry(telemetry);
            idle();
        }
        if(opModeIsActive()){
            ReadyPeripherals();
            while(opModeIsActive()){
                if(!ExecuteFlag) {
                    AutoRingLaunch();
                    switch (ringData){
                        case ZERO:
                            MoveWithEncoder(285, 3);
                            DeployArm();
                            break;
                        case ONE:
                            MoveWithEncoder(285, 0);
                            MoveWithEncoder(285, 1);
                            DeployArm();
                            MoveWithEncoder(285, 2);
                            break;
                        case FOUR:
                            MoveWithEncoder(285, 3);
                            MoveWithEncoder(866, 0);
                            DeployArm();
                            MoveWithEncoder(866, 2);
                            break;
                    }
                    RetractArm();
                    ExecuteFlag=true;
                }
                else idle();
            }
        }
    }
}