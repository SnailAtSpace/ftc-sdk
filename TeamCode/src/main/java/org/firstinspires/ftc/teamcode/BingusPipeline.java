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
    Mat scanArea = new Mat();
    Mat frame = new Mat();
    double minColour;
    double avgR,avgG,avgB;
    final int xL=330, y=17;
    final int offsetX=30,offsetY=60;
    Point ptA =new Point(xL,y), ptB = new Point(xL+offsetX,y+offsetY);
    private volatile int zone = 0;
    private volatile Mat img;

    public void init(Mat firstFrame){
        scanArea = firstFrame.submat(new Rect(ptA, ptB));
    }

    @Override
    public Mat processFrame(Mat input) {
        avgR = Core.mean(scanArea).val[0];
        avgG = Core.mean(scanArea).val[1];
        avgB = Core.mean(scanArea).val[2];
        minColour = Math.min(avgR,Math.min(avgG,avgB));
        if(minColour == avgR){ //cyan
            zone = 1;
        }
        else if(minColour == avgG){
            zone = 2;
        }
        else if(minColour == avgB){
            zone = 3;
        }
        Imgproc.rectangle(input,ptA,ptB,new Scalar(255*(minColour==avgR?1:0),255*(minColour==avgG?1:0),255*(minColour==avgB?1:0)),2);
        return input;
    }

    public int getAnal() {
        return zone;
    }

    public double[] getVals(double left, double center, double right){
        return new double[]{left,center,right};
    }

    @SuppressLint("DefaultLocale")
    public int ComposeTelemetry(Telemetry telemetry){
        telemetry.addData("Best guess of duck position: ", getAnal());
        telemetry.addData("Vals: ",String.format("%1$.2f %2$.2f %3$.2f", avgR,avgG,avgB));
        telemetry.update();
        return getAnal();
    }
}

