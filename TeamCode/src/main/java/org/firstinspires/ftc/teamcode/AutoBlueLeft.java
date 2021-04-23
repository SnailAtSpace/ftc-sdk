package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous BLUE Edition: Left Edition",preselectTeleOp = "Bingus Controller Test")
public class AutoBlueLeft extends CommonOpMode{
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode(){
        Initialize(hardwareMap,true, BingusPipeline.StartLine.LEFT);
        color = Color.BLUE;
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
                            MoveWithEncoder(200, 2);
                            OrientToDegrees(-90);
                            DeployArm();
                            MoveWithEncoder(175, 2);
                            RetractArm();
                            OrientToDegrees(0);
                            break;
                        case ONE:
                            MoveWithEncoder(1200, 2);
                            OrientToDegrees(90);
                            DeployArm();
                            MoveWithEncoder(100, 2);
                            RetractArm();
                            OrientToDegrees(0);
                            MoveWithEncoder(300, 0);
                            OrientToDegrees(0);
                            break;
                        case FOUR:
                            MoveWithEncoder(1050, 2);
                            OrientToDegrees(-120);
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