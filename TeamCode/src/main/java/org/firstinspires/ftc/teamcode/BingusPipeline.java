package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

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
        LEFT,
        CENTER,
        RIGHT,
        UNDEFINED
    }
    Mat left = new Mat();
    Mat center = new Mat();
    Mat right = new Mat();
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    double minCb;
    double avgL,avgC,avgR;
    final int sideOffset=0;
    final int xL=0+sideOffset, y=175, xC=305, xR=610-sideOffset;
    final int offsetX=30,offsetY=7;
    Point leftA=new Point(xL,y),leftB = new Point(xL+offsetX,y+offsetY);
    Point centerA=new Point(xC,y),centerB = new Point(xC+offsetX,y+offsetY);
    Point rightA=new Point(xR,y),rightB = new Point(xR+offsetX,y+offsetY);
    private volatile BingusPipeline.RandomizationFactor position = RandomizationFactor.UNDEFINED;

    void inputToYCrCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
    }

    public void init(Mat firstFrame){
        inputToYCrCb(firstFrame);
        left = YCrCb.submat(new Rect(leftA, leftB));
        center = YCrCb.submat(new Rect(centerA, centerB));
        right = YCrCb.submat(new Rect(rightA,rightB));
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToYCrCb(input);
        avgL = Core.mean(left).val[2];
        avgC = Core.mean(center).val[2];
        avgR = Core.mean(right).val[2];
        minCb = Math.min(avgL,Math.min(avgC,avgR));
        if(minCb == avgL){
            position = RandomizationFactor.LEFT;
        }
        else if(minCb == avgC){
            position = RandomizationFactor.CENTER;
        }
        else if(minCb == avgR){
            position = RandomizationFactor.RIGHT;
        }
        Imgproc.rectangle(input, leftA, leftB, new Scalar(255*BTI(position!=RandomizationFactor.LEFT),255*BTI(position==RandomizationFactor.LEFT),0), 2);
        Imgproc.rectangle(input, centerA, centerB, new Scalar(255*BTI(position!=RandomizationFactor.CENTER),255*BTI(position==RandomizationFactor.CENTER),0), 2);
        Imgproc.rectangle(input, rightA, rightB, new Scalar(255*BTI(position!=RandomizationFactor.RIGHT),255*BTI(position==RandomizationFactor.RIGHT),0), 2);
        return input;
    }

    public BingusPipeline.RandomizationFactor getAnal() {
        return position;
    }

    public double[] getVals(double left, double center, double right){
        return new double[]{left,center,right};
    }

    @SuppressLint("DefaultLocale")
    public RandomizationFactor ComposeTelemetry(Telemetry telemetry){
        telemetry.addData("Best guess of duck position: ", getAnal());
        telemetry.addData("Vals: ",String.format("%1$.2f %2$.2f %3$.2f", avgL,avgC,avgR));
        telemetry.update();
        return getAnal();
    }

    public int BTI(boolean b){return b?1:0;}
}

