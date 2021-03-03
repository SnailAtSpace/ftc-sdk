package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name="Autonomous, but it's so bad")

public class AutonomousTest extends OpMode {
    OpenCvCamera webcam;
    BingusPipeline pipeline;
    Boolean ExecuteFlag;
    DcMotor FRmotor;DcMotor RRmotor;DcMotor FLmotor;DcMotor RLmotor;DcMotor Worm;Servo Grabber;DcMotor Flywheel;Servo Pushrod;
    public BingusPipeline.RandomizationFactor ringData;
    public ElapsedTime whenAreWe = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    @Override
    public void init(){
        FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");
        RRmotor = hardwareMap.get(DcMotor.class, "RRmotor");
        FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
        RLmotor = hardwareMap.get(DcMotor.class, "RLmotor");
        Flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        Worm = hardwareMap.get(DcMotor.class, "worm");
        Grabber = hardwareMap.get(Servo.class,"grabber");
        Pushrod = hardwareMap.get(Servo.class,"pushrod");
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
        try {
            Thread.sleep(50);
        } catch (InterruptedException ignored) {
        }
    }
    @SuppressLint("DefaultLocale")
    @Override
    public void init_loop(){
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
    @Override
    public void start(){
        Grabber.scaleRange(0.2,0.66);
        Grabber.setPosition(0);
        whenAreWe.reset();
        ExecuteFlag=false;
    }
    @Override
    public void loop(){
        if(!ExecuteFlag) {
            DeployArm();
            MoveByMillimetres(2032, 2);
            MoveByMillimetres(290,3);
            LaunchSeveralRings(3);
            if (ringData == BingusPipeline.RandomizationFactor.ONE) {
                MoveByMillimetres(290, 3);
                MoveByMillimetres(600, 2);
                Grabber.setPosition(1);
                MoveByMillimetres(600, 0);
            } else {
                MoveByMillimetres(290, 1);
                if (ringData == BingusPipeline.RandomizationFactor.ZERO) {
                    Grabber.setPosition(1);
                } else {
                    MoveByMillimetres(1200, 2);
                    Grabber.setPosition(1);
                    MoveByMillimetres(1200, 0);
                }
            }
            ExecuteFlag=true;
        }
        else try { Thread.sleep(50); } catch (InterruptedException ignored) {}
    }
    public static class BingusPipeline extends OpenCvPipeline {
        public enum RandomizationFactor {
            ZERO,
            ONE,
            FOUR
        }
        Mat region1_Cb = new Mat();
        Mat region2_Cb = new Mat();
//        Mat region1_Cr;
//        Mat region2_Cr;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
//        Mat Cr = new Mat();
        float avgB1;
        float avgB2;
        final int xL=20,yL=140,xH=20,yH=100;
        final int offsetX=60,offsetY=10;
        Point regLowerA=new Point(xL,yL), regHigherA=new Point(xH,yH);
        Point regLowerB=new Point(xL+offsetX, yL+offsetY), regHigherB=new Point(xH+offsetX, yH+offsetY);
        private volatile RandomizationFactor position;
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }
//        void inputToCr(Mat input){
//            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//            Core.extractChannel(YCrCb, Cr, 1);
//        }
        @Override
        public void init(Mat firstFrame){
            inputToCb(firstFrame);
            region1_Cb = Cb.submat(new Rect(regLowerA, regLowerB));
            region2_Cb = Cb.submat(new Rect(regHigherA, regHigherB));
//            region1_Cr = Cr.submat(new Rect(regLowerA, regLowerB));
//            region2_Cr = Cr.submat(new Rect(regHigherA, regHigherB));

        }
        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);
            avgB1 = (float) Core.mean(region1_Cb).val[0];
            avgB2 = (float) Core.mean(region2_Cb).val[0];
            Imgproc.rectangle(input, regLowerA, regLowerB, new Scalar(255,0,0), 2);
            Imgproc.rectangle(input, regHigherA, regHigherB, new Scalar(255,0,0), 2);
            if(avgB1<=110&&avgB2<=110){
                position = RandomizationFactor.FOUR;
                Imgproc.rectangle(input, regLowerA, regLowerB, new Scalar(0,0,255), 2);
                Imgproc.rectangle(input, regHigherA, regHigherB, new Scalar(0,0,255), 2);
            }                                  //both orange
            else if(avgB1<=110&&avgB2>110){
                position = RandomizationFactor.ONE;
                Imgproc.rectangle(input, regLowerA, regLowerB, new Scalar(0,0,255), 2);
                Imgproc.rectangle(input, regHigherA, regHigherB, new Scalar(255,0,0), 2);
            }
            else {
                position = RandomizationFactor.ZERO;
            }
            return input;
        }
        public RandomizationFactor getAnal() {
            return position;
        }
        public float getLower() {
            return avgB1;
        }
        public float getHigher() {
            return avgB2;
        }
    }
    public void MoveByMillimetres(float millis,int direction){
        //direction counted from 0, being backwards, counterclockwise
        //0=backward, 1=left, 2=forward, 3=right
        ElapsedTime localTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(localTime.time()<=millis*1.131889763779527) { //what the fuck am i doing
            RLmotor.setPower(1*Math.signum((direction-1)*2-1));
            RRmotor.setPower(1*Math.signum(direction%3*2-1));
            FLmotor.setPower(1*Math.signum(direction%3*2-1));
            RLmotor.setPower(1*Math.signum((direction-1)*2-1));
        }
        RRmotor.setPower(0);
        FLmotor.setPower(0);
        RLmotor.setPower(0);
        RLmotor.setPower(0);
    }
    public void DeployArm(){
        whenAreWe.reset();
        Worm.setPower(-1);
        while(whenAreWe.time()<=100){}
        Worm.setPower(0);
    }
    public void LaunchSeveralRings(int amount){
        ElapsedTime localTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        Flywheel.setPower(1);
        while (localTime.time() <= 1000) {}
        Pushrod.setPosition(1);
        while (localTime.time() <= 1100) {}
        Pushrod.setPosition(0);
        for(int i=0;i<=amount--;i++){
            localTime.reset();
            while (localTime.time() <= 500) {}
            Flywheel.setPower(1);
            while (localTime.time() <= 1500) {}
            Pushrod.setPosition(1);
            while (localTime.time() <= 1600) {}
            Pushrod.setPosition(0);
        }
        Flywheel.setPower(0);
    }
}
