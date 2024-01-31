package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Test me!")
public class AutoSomething extends CommonOpMode{
    // TODO: определение, парковка
    // рандомизация -> опа ставим жёлтый на спот -> опа ставим фиолетовый на рандомизацию -> паркуемся
    @Override
    public void runOpMode() throws InterruptedException {
        Initialize(hardwareMap);
        camUp();
        while(opModeInInit()){
            rand = pipeline.ComposeTelemetry(telemetry);
        }
        switch(rand){
            case 0:
                //TODO: add the trajectories
            case 1:

            case 2:

        }
    }
}