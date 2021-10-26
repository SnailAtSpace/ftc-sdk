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
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    float avgB1;
    float avgB2;
    final int xL=15,yL=230,yH=170,xLr=555;
    final int offsetX=30,offsetY=7;
    final int sideOffset=10;
    Point regLowerA=new Point(xL,yL), regHigherA=new Point(xL,yH),regLowerAr=new Point(xLr,yL-sideOffset), regHigherAr=new Point(xLr,yH-sideOffset);
    Point regLowerB=new Point(xL+offsetX, yL+offsetY), regHigherB=new Point(xL+offsetX, yH+offsetY);
    Point regLowerBr=new Point(xLr+offsetX, yL+offsetY-sideOffset), regHigherBr=new Point(xLr+offsetX, yH+offsetY-sideOffset);
    Point LowerA,LowerB,HigherA,HigherB;
    private volatile BingusPipeline.RandomizationFactor position;
    private volatile BingusPipeline.StartLine side;

    void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    public void setSide(StartLine side){
        this.side = side;
    }

    public void init(Mat firstFrame){
        inputToCb(firstFrame);
        if(side == StartLine.RIGHT){
            LowerA=regLowerA;
            LowerB=regLowerB;
            HigherA=regHigherA;
            HigherB=regHigherB;
        }
        else if(side == StartLine.LEFT){
            LowerA=regLowerAr;
            LowerB=regLowerBr;
            HigherA=regHigherAr;
            HigherB=regHigherBr;
        }
        region1_Cb = Cb.submat(new Rect(LowerA, LowerB));
        region2_Cb = Cb.submat(new Rect(HigherA, HigherB));
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);
        avgB1 = (float) Core.mean(region1_Cb).val[0];
        avgB2 = (float) Core.mean(region2_Cb).val[0];
        Imgproc.rectangle(input, LowerA, LowerB, new Scalar(255,255*BTI(avgB1<=110),0), 2);
        Imgproc.rectangle(input, HigherA, HigherB, new Scalar(255,255*BTI(avgB2<=110),0), 2);
        if(avgB1<=110&&avgB2<=110){
            position = BingusPipeline.RandomizationFactor.FOUR;
        }
        else if(avgB1<=110&&avgB2>110){
            position = BingusPipeline.RandomizationFactor.ONE;
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

    public RandomizationFactor ComposeTelemetry(Telemetry telemetry){
        telemetry.addData("Best guess of ring amount: ", getAnal());
        telemetry.addData("Lower: ", getLower());
        telemetry.addData("Higher: ", getHigher());
        telemetry.update();
        return getAnal();
    }

    public int BTI(boolean b){return b?1:0;}
}

