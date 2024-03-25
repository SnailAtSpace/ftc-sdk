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
    Mat scanAreaLeft = new Mat(),scanAreaCenter = new Mat(),scanAreaRight = new Mat();
    Mat YCrCb = new Mat();
    Mat Ctgt = new Mat();
    Mat frame = new Mat();
    double maxArea;
    double avgL,avgC,avgR;
    int isRed;
    int xL=0, xC=240, xR=400, y=0; // TODO: отрегулировать начальные точки и размеры
    final int offsetX=320,offsetY=180; // размер полей с отсчётом от нижней(?) левой точки
    Rect rL, rC, rR;
    private volatile int zone = 0;
    private volatile Mat img;

    public BingusPipeline(boolean isRed){
        this.isRed = isRed?1:2;
        rL = new Rect(new Point(xL, y), new Point(xL+offsetX, y+240));
        rC = new Rect(new Point(xC, y), new Point(xC+offsetX, y+160));
        rR = new Rect(new Point(xR, y), new Point(xR+offsetX, y+240));
    }

    public void init(Mat firstFrame){
        inputToCb(firstFrame);
        scanAreaLeft = Ctgt.submat(rL);
        scanAreaCenter = Ctgt.submat(rC);
        scanAreaRight = Ctgt.submat(rR);
    }

    void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Ctgt, isRed);
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);
        avgL = Core.mean(scanAreaLeft).val[0];
        avgC = Core.mean(scanAreaCenter).val[0];
        avgR = Core.mean(scanAreaRight).val[0];
        maxArea = Math.max(avgL, Math.max(avgC, avgR));
        if(maxArea == avgL){
            zone = 0;
        }
        if(maxArea == avgC){
            zone = 1;
        }
        if(maxArea == avgR){
            zone = 2;
        }
        draw(rL, input);
        draw(rC, input);
        draw(rR, input);
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
        telemetry.addData("Vals: ",String.format("%1$.2f %2$.2f %3$.2f", avgL, avgC, avgR));
        telemetry.update();
        return getAnal();
    }

    private void draw(Rect r, Mat i){
        Imgproc.rectangle(i,r,new Scalar(255, 255, 255),2);
    }
}

