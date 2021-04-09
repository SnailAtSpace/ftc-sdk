package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BingusPipeline extends OpenCvPipeline {
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
    final int xL=0,yL=151,xH=0,yH=123;
    final int offsetX=60,offsetY=7;
    Point regLowerA=new Point(xL,yL), regHigherA=new Point(xH,yH);
    Point regLowerB=new Point(xL+offsetX, yL+offsetY), regHigherB=new Point(xH+offsetX, yH+offsetY);
    private volatile BingusPipeline.RandomizationFactor position;
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
        if(avgB1<=111&&avgB2<=111){
            position = BingusPipeline.RandomizationFactor.FOUR;
            Imgproc.rectangle(input, regLowerA, regLowerB, new Scalar(255,255,0), 2);
            Imgproc.rectangle(input, regHigherA, regHigherB, new Scalar(255,255,0), 2);
        }                                  //both orange
        else if(avgB1<=111&&avgB2>111){
            position = BingusPipeline.RandomizationFactor.ONE;
            Imgproc.rectangle(input, regLowerA, regLowerB, new Scalar(255,255,0), 2);
            Imgproc.rectangle(input, regHigherA, regHigherB, new Scalar(255,0,0), 2);
        }
        else {
            position = BingusPipeline.RandomizationFactor.ZERO;
        }
        return input;
    }
    public BingusPipeline.RandomizationFactor getAnal() {
        return position;
    }
    public float getLower() {
        return avgB1;
    }
    public float getHigher() {
        return avgB2;
    }
    public void ComposeTelemetry(Telemetry telemetry){
        RandomizationFactor ringData = getAnal();
        telemetry.addData("Best guess of ring amount: ", ringData);
        telemetry.addData("Lower: ", getLower());
        telemetry.addData("Higher: ", getHigher());
        telemetry.update();
    }
}

