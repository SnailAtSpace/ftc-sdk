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
    public enum StartLine {
        LEFT,
        RIGHT
    }
    Mat region1_Cb = new Mat();
    Mat region2_Cb = new Mat();
    Mat region1r_Cb = new Mat();
    Mat region2r_Cb = new Mat();
    //        Mat region1_Cr;
//        Mat region2_Cr;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    //        Mat Cr = new Mat();
    float avgB1;
    float avgB2;
    float avgB1r;
    float avgB2r;
    final int xL=15,yL=172,yH=144,xLr=275;
    final int offsetX=30,offsetY=7;
    Point regLowerA=new Point(xL,yL), regHigherA=new Point(xL,yH),regLowerAr=new Point(xLr,yL), regHigherAr=new Point(xLr,yH);;
    Point regLowerB=new Point(xL+offsetX, yL+offsetY), regHigherB=new Point(xL+offsetX, yH+offsetY);
    Point regLowerBr=new Point(xLr+offsetX, yL+offsetY), regHigherBr=new Point(xLr+offsetX, yH+offsetY);
    private volatile BingusPipeline.RandomizationFactor position;
    private volatile BingusPipeline.StartLine side;
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
        region1r_Cb = Cb.submat(new Rect(regLowerAr, regLowerBr));
        region2r_Cb = Cb.submat(new Rect(regHigherAr, regHigherBr));
//            region1_Cr = Cr.submat(new Rect(regLowerA, regLowerB));
//            region2_Cr = Cr.submat(new Rect(regHigherA, regHigherB));

    }
    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);
        avgB1 = (float) Core.mean(region1_Cb).val[0];
        avgB2 = (float) Core.mean(region2_Cb).val[0];
        avgB1r = (float) Core.mean(region1r_Cb).val[0];
        avgB2r = (float) Core.mean(region2r_Cb).val[0];
        Imgproc.rectangle(input, regLowerA, regLowerB, new Scalar(255,255*BTI(avgB1<=111),0), 2);
        Imgproc.rectangle(input, regHigherA, regHigherB, new Scalar(255,255*BTI(avgB2<=111),0), 2);
        Imgproc.rectangle(input, regLowerAr, regLowerBr, new Scalar(255,255*BTI(avgB1r<=111),0), 2);
        Imgproc.rectangle(input, regHigherAr, regHigherBr, new Scalar(255,255*BTI(avgB2r<=111),0), 2);
        if(avg(avgB1,avgB2)<avg(avgB1r,avgB2r)){            side = StartLine.RIGHT;
            if(avgB1<=111&&avgB2<=111){
                position = BingusPipeline.RandomizationFactor.FOUR;
            }
            else if(avgB1<=111&&avgB2>111){
                position = BingusPipeline.RandomizationFactor.ONE;
            }
            else {
                position = BingusPipeline.RandomizationFactor.ZERO;
            }
        }
        else{
            side = StartLine.LEFT;
            if(avgB1r<=111&&avgB2r<=111){
                position = BingusPipeline.RandomizationFactor.FOUR;
            }
            else if(avgB1r<=111&&avgB2r>111){
                position = BingusPipeline.RandomizationFactor.ONE;
            }
            else {
                position = BingusPipeline.RandomizationFactor.ZERO;
            }
        }
        return input;
    }
    public BingusPipeline.RandomizationFactor getAnal() {
        return position;
    }
    public BingusPipeline.StartLine getSide() {
        return side;
    }
    public float getLower() {
        return avgB1;
    }
    public float getHigher() {
        return avgB2;
    }
    public float getLowerR() {
        return avgB1r;
    }
    public float getHigherR() {
        return avgB2r;
    }
    public RandomizationFactor ComposeTelemetry(Telemetry telemetry){
        telemetry.addData("Best guess of ring amount: ", getAnal());
        telemetry.addData("Best guess of side: ", getSide());
        telemetry.addData("Lower: ", getLower());
        telemetry.addData("Higher: ", getHigher());
        telemetry.addData("Lower Right: ", getLowerR());
        telemetry.addData("Higher Right: ", getHigherR());
        telemetry.addData("AR",avg(avgB1,avgB2));
        telemetry.addData("AL",avg(avgB1r,avgB2r));
        telemetry.update();
        return getAnal();
    }
    public int BTI(boolean b){return b?1:0;}
    public double avg(double a,double b){return (a+b)/2.0;}
}

