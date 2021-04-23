package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous BLUE Edition: Right Edition",preselectTeleOp = "Bingus Controller Test")
public class AutoBlueEnc extends CommonOpMode{
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode(){
        Initialize(hardwareMap,true, BingusPipeline.StartLine.RIGHT);
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
                            MoveWithEncoder(75, 2);
                            OrientToDegrees(-90);
                            MoveWithEncoder(150, 0);
                            DeployArm();
                            MoveWithEncoder(800, 2);
                            RetractArm();
                            OrientToDegrees(0);
                            break;
                        case ONE:
                            MoveWithEncoder(500, 2);
                            OrientToDegrees(-90);
                            DeployArm();
                            MoveWithEncoder(650, 2);
                            RetractArm();
                            OrientToDegrees(0);
                            MoveWithEncoder(100, 0);
                            OrientToDegrees(0);
                            break;
                        case FOUR:
                            MoveWithEncoder(300, 2);
                            OrientToDegrees(-135);
                            MoveWithEncoder(225, 0);
                            DeployArm();
                            MoveWithEncoder(825, 2);
                            RetractArm();
                            OrientToDegrees(0);
                            MoveWithEncoder(100, 0);
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