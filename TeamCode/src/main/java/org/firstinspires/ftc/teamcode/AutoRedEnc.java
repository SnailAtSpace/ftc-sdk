package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous RED edition: Right Edition",preselectTeleOp = "Bingus Controller Test")
public class AutoRedEnc extends CommonOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode(){
        Initialize(hardwareMap,true,BingusPipeline.StartLine.RIGHT);
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
                            MoveWithEncoder(375, 2);
                            OrientToDegrees(90);
                            MoveWithEncoder( 150, 2);
                            DeployArm();
                            RetractArm();
                            OrientToDegrees(0);
                            MoveWithEncoder(75,0);
                            OrientToDegrees(0);
                            break;
                        case ONE:
                            MoveWithEncoder(550, 2);
                            OrientToDegrees(-90);
                            MoveWithEncoder(75, 2);
                            DeployArm();
                            RetractArm();
                            OrientToDegrees(0);
                            MoveWithEncoder(150, 0);
                            OrientToDegrees(0);
                            break;
                        case FOUR:
                            MoveWithEncoder(1350, 2);
                            OrientToDegrees(100);
                            OrientToDegrees(145);
                            MoveWithEncoder(75, 2);
                            DeployArm();
                            MoveWithEncoder(50, 2);
                            RetractArm();
                            OrientToDegrees(0);
                            MoveWithEncoder(275, 0);
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