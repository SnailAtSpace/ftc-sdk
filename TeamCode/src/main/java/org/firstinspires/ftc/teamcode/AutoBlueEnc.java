package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Autonomous BLUE Edition: with BONUS encoders!",preselectTeleOp = "Bingus Controller Test")
public class AutoBlueEnc extends LinearOpMode {
    OpenCvCamera webcam;
    BingusPipeline pipeline;
    Boolean ExecuteFlag;
    DcMotor FRmotor, RRmotor, FLmotor, RLmotor, Worm, Flywheel, Collector;
    Servo Grabber, Pushrod;
    public BingusPipeline.RandomizationFactor ringData;
    public ElapsedTime whenAreWe = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode(){
        FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");                        //Hardware mapping and declaration of devices
        RRmotor = hardwareMap.get(DcMotor.class, "RRmotor");
        FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
        RLmotor = hardwareMap.get(DcMotor.class, "RLmotor");
        Flywheel = hardwareMap.get(DcMotor.class, "FWmotor");
        Worm = hardwareMap.get(DcMotor.class, "Wmotor");
        Grabber = hardwareMap.get(Servo.class,"Gservo");
        Pushrod = hardwareMap.get(Servo.class,"Pservo");
        Collector = hardwareMap.get(DcMotor.class,"Cmotor");
        FRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Worm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new BingusPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
        telemetry.addLine("Waiting for start");
        telemetry.update();
        FLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        while((!isStarted())&&(!isStopRequested())){
            ringData=pipeline.getAnal();
            telemetry.addData("Best guess of ring amount: ",ringData);
            telemetry.addData("Lower: ",pipeline.getLower());
            telemetry.addData("Higher: ",pipeline.getHigher());
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();
            try {
                Thread.sleep(50);
            } catch (InterruptedException ignored) {
            }
        }
        if(opModeIsActive()){
            Grabber.scaleRange(0.16,0.66);
            Pushrod.scaleRange(0.19,0.25);
            Pushrod.setPosition(0);
            whenAreWe.reset();
            ExecuteFlag=false;
            while(opModeIsActive()){
                if(!ExecuteFlag) {
                    Grabber.setPosition(0);
                    MoveWithEncoder(1890, 2);
                    TurnBySeconds(90,1);
                    LaunchSeveralRings(3);
                    TurnBySeconds(90,0);
//                    if(ringData != BingusPipeline.RandomizationFactor.ZERO) {
//                        MoveWithEncoder(550, 3);
//                        Collector.setPower(0.75);
//                        MoveWithEncoder(870,0);
//                        sleep(100);
//                        if(ringData== BingusPipeline.RandomizationFactor.FOUR){MoveWithEncoder(100,0);MoveWithEncoder(100,2);}
//                        MoveWithEncoder(860,2);
//                        Collector.setPower(0);
//                        MoveWithEncoder(550,1);
//                        TurnBySeconds(90,1);
//                        if(ringData==BingusPipeline.RandomizationFactor.ONE)LaunchSeveralRings(1);
//                        else LaunchSeveralRings(3);
//                        TurnBySeconds(90,0);
//                    }
                    TurnBySeconds(1450,1);
                    if (ringData == BingusPipeline.RandomizationFactor.ONE) {
                        MoveWithEncoder(600, 0);
                        MoveWithEncoder(400, 1);
                        DeployArm();
                        Grabber.setPosition(1);
                        sleep(2000);
                        MoveWithEncoder(600, 2);
                    } else {
                        if (ringData == BingusPipeline.RandomizationFactor.ZERO) {
                            MoveWithEncoder(1200, 1);
                            DeployArm();
                            Grabber.setPosition(1);
                            sleep(2000);
                        } else {
                            MoveWithEncoder(1200, 0);
                            MoveWithEncoder(1200, 1);
                            DeployArm();
                            Grabber.setPosition(1);
                            sleep(2000);
                            MoveWithEncoder(1200, 2);
                        }
                    }
                    ExecuteFlag=true;
                }
                else idle();
            }
        }
    }
//    public void MoveByMillimetres(float millis,int direction){
//        //direction counted from 0, being backwards, counterclockwise
//        //0=backward, 1=right, 2=forward, 3=left
//        ElapsedTime localTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//        while(localTime.time()<=millis*1.135) { //what the fuck am i doing
//            RLmotor.setPower(Math.signum((direction-1)*2-1));
//            RRmotor.setPower(Math.signum(direction%3*2-1));
//            FLmotor.setPower(Math.signum(direction%3*2-1));
//            FRmotor.setPower(Math.signum((direction-1)*2-1));
//        }
//        RLmotor.setPower(0);
//        RRmotor.setPower(0);
//        FLmotor.setPower(0);
//        FRmotor.setPower(0);
//        sleep(250);
//    }
    public void MoveWithEncoder(int millis,int direction){
        //direction counted from 0, being backwards, counterclockwise
        //0=backward, 1=right, 2=forward, 3=left
        FRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRmotor.setTargetPosition(millis);
        FRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRmotor.setPower(Math.signum((direction-1)*2-1));
        RLmotor.setPower(Math.signum((direction-1)*2-1));
        RRmotor.setPower(Math.signum(direction%3*2-1));
        FLmotor.setPower(Math.signum(direction%3*2-1));
        FRmotor.setPower(0);
        RLmotor.setPower(0);
        RRmotor.setPower(0);
        FLmotor.setPower(0);
        sleep(250);
    }
    public void DeployArm(){
        Worm.setPower(-1);
        sleep(2100);
        Worm.setPower(0);
    }
    public void LaunchSeveralRings(int amount){
        ElapsedTime localTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        Flywheel.setPower(1);
        while (localTime.time() <= 3000) {}
        Pushrod.setPosition(1);
        while (localTime.time() <= 3100) {}
        Pushrod.setPosition(0);
        for(int i=0;i<=amount--;i++){
            localTime.reset();
            while (localTime.time() <= 2000) {}
            Pushrod.setPosition(1);
            while (localTime.time() <= 2100) {}
            Pushrod.setPosition(0);
        }
        Flywheel.setPower(0);
    }
    public void TurnBySeconds(int millis,int direction){
        ElapsedTime localTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(localTime.time()<=millis) { //what the fuck am i doing
            RLmotor.setPower(1-direction*2);
            RRmotor.setPower(direction*2-1);
            FLmotor.setPower(1-direction*2);
            FRmotor.setPower(direction*2-1);
        }
        RLmotor.setPower(0);
        RRmotor.setPower(0);
        FLmotor.setPower(0);
        FRmotor.setPower(0);
        sleep(100);
    }
}

