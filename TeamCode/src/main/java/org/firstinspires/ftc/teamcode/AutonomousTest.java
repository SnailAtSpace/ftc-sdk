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

@Autonomous(name="Autonomous, but it's bad")

public class AutonomousTest extends OpMode {
    OpenCvCamera webcam;
    BingusPipeline pipeline;
    public BingusPipeline.RandomizationFactor ringAmount;
    ElapsedTime whenAreWe = new ElapsedTime();
    DcMotor FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");                    //Hardware mapping and declaration of devices
    DcMotor RRmotor = hardwareMap.get(DcMotor.class, "RRmotor");
    DcMotor FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
    DcMotor RLmotor = hardwareMap.get(DcMotor.class, "RLmotor");
    DcMotor Worm = hardwareMap.get(DcMotor.class, "worm");//TODO:Properly unfold the robot according to wobble placement and robot size
    Servo Grabber = hardwareMap.get(Servo.class,"grabber");
    @Override
    public void init(){
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
    }
    @SuppressLint("DefaultLocale")
    @Override
    public void init_loop(){
        ringAmount=pipeline.getAnal();
        telemetry.addData("Best guess of ring amount: ",ringAmount);
        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
        telemetry.update();
    }
    @Override
    public void start(){
        Grabber.scaleRange(0.22,0.66);
        Grabber.setPosition(1);
        whenAreWe.reset();
        Worm.setPower(1);
        while(whenAreWe.time()<=1000)
        Worm.setPower(0);
        Grabber.setPosition(0);
        MoveByTicks(6600,2);
        if(ringAmount==BingusPipeline.RandomizationFactor.ZERO||ringAmount==BingusPipeline.RandomizationFactor.FOUR) {
            MoveByTicks(1300, 1);
            if(ringAmount==BingusPipeline.RandomizationFactor.ZERO){
                MoveByTicks(300,2);
                Grabber.setPosition(1);
                MoveByTicks(300,0);
            }
            else {
                MoveByTicks(3000,2);
                Grabber.setPosition(1);
                MoveByTicks(3000,0);
            }
            MoveByTicks(1300, 3);
        }
        else {
            MoveByTicks(1300,3);
            MoveByTicks(2000,2);
            Grabber.setPosition(1);
            MoveByTicks(1300,1);
            MoveByTicks(2000,0);
        }
        MoveByTicks(6600,0);
    }
    @Override
    public void loop(){
        try {
            Thread.sleep(50);
        } catch (InterruptedException ignored) {
        }
    }
    public static class BingusPipeline extends OpenCvPipeline {
        public enum RandomizationFactor {
            ZERO,
            ONE,
            FOUR
        }
        Mat region1_Cb;
        Mat region2_Cb;
        Mat region1_Cr;
        Mat region2_Cr;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        Mat Cr = new Mat();
        int avgB1,avgR1;
        int avgB2,avgR2;
        int offsetX=60,offsetY=15;
        Point regLowerA =new Point(40,144), regHigherA =new Point(40,114);//FIXME:Fix submat size according to images from webcam and ring placement
        Point regLowerB =new Point(regLowerA.x+offsetX, regLowerA.y+offsetY), regHigherB =new Point(regHigherA.x+offsetX, regHigherA.y+offsetY);
        private volatile RandomizationFactor position = RandomizationFactor.ZERO;
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }
        void inputToCr(Mat input){
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cr, 3);
        }
        @Override
        public void init(Mat firstFrame){
            inputToCb(firstFrame);
            region1_Cb = Cb.submat(new Rect(regLowerA, regLowerB));
            region2_Cb = Cb.submat(new Rect(regHigherA, regHigherB));
            region1_Cr = Cr.submat(new Rect(regLowerA, regLowerB));
            region2_Cr = Cr.submat(new Rect(regHigherA, regHigherB));
        }
        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);
            inputToCr(input);
            avgB1 = (int) Core.mean(region1_Cb).val[0];
            avgB2 = (int) Core.mean(region2_Cb).val[0];
            avgR1 = (int) Core.mean(region1_Cr).val[0];
            avgR2 = (int) Core.mean(region2_Cr).val[0];
            Imgproc.rectangle(input, regLowerA, regLowerB, new Scalar(255,0,0), 2);
            Imgproc.rectangle(input, regHigherA, regHigherB, new Scalar(255,0,0), 2);
            if(avgB1<=-0.5&&avgR1>=0.5&&avgB2<=-0.5&&avgR2>=0.5){
                position = RandomizationFactor.FOUR;
            }                                  //both orange
            else if(avgB1<=-0.5&&avgR1>=0.5&&avgB2>-0.5&&avgR2<0.5){
                position = RandomizationFactor.ONE;
            }
            else position = RandomizationFactor.ZERO;
            return input;
        }
        public RandomizationFactor getAnal(){
            return position;
        }
    }
    public void MoveByTicks(int ticks,int direction){
        FRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//TODO:Adjust movement of robot according to wobble placement and field measurements on-site.
        FRmotor.setTargetPosition((int)(ticks*Math.signum((direction-1)*2-1)));
        FRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRmotor.setPower(1);
        while(FRmotor.isBusy()) {
            RRmotor.setPower(1*Math.signum(direction%3*2-1));
            FLmotor.setPower(1*Math.signum(direction%3*2-1));
            RLmotor.setPower(1*Math.signum((direction-1)*2-1));
        }
        RRmotor.setPower(0);
        FLmotor.setPower(0);
        RLmotor.setPower(0);
    }
}
