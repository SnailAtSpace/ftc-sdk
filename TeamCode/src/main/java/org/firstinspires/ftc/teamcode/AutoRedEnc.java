package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous RED edition: Right Edition",preselectTeleOp = "Bingus Controller Test")
public class AutoRedEnc extends CommonOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode(){
        Initialize(hardwareMap,true,BingusPipeline.StartLine.RIGHT);
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
                            MoveWithEncoder(500, 2);
                            OrientToDegrees(90);
                            DeployArm();
                            MoveWithEncoder(175, 2);
                            RetractArm();
                            OrientToDegrees(0);
                            MoveWithEncoder(100,0);
                            OrientToDegrees(0);
                            break;
                        case ONE:
                            MoveWithEncoder(800, 2);
                            OrientToDegrees(-90);
                            MoveWithEncoder(150, 2);
                            DeployArm();
                            MoveWithEncoder(100, 2);
                            RetractArm();
                            OrientToDegrees(0);
                            MoveWithEncoder(200, 0);
                            OrientToDegrees(0);
                            break;
                        case FOUR:
                            MoveWithEncoder(1300, 2);
                            OrientToDegrees(130);
                            MoveWithEncoder(50, 0);
                            DeployArm();
                            MoveWithEncoder(175, 2);
                            RetractArm();
                            OrientToDegrees(0);
                            MoveWithEncoder(300, 0);
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