package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous BLUE Edition: with BONUS encoders!",preselectTeleOp = "Bingus Controller Test")
public class AutoBlueEnc extends CommonOpMode{
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
                            MoveWithEncoder(1200, 1);
                            DeployArm();
                            break;
                        case ONE:
                            MoveWithEncoder(600, 0);
                            MoveWithEncoder(400, 1);
                            DeployArm();
                            MoveWithEncoder(600, 2);
                            break;
                        case FOUR:
                            MoveWithEncoder(1200, 0);
                            MoveWithEncoder(1200, 1);
                            DeployArm();
                            MoveWithEncoder(1200, 2);
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