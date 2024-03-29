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
    Mat scanAreaLeft = new Mat(), scanAreaCenter = new Mat();
    Mat YCrCb = new Mat();
    Mat Ctgt = new Mat();
    Mat frame = new Mat();
    double maxArea;
    double avgL, avgC;
    int isRed;
    int xL = 133, xC = 221 + 320, y = 160, yC = 120; // TODO: отрегулировать начальные точки и размеры
    final int offsetX = 20, offsetY = 20; // размер полей с отсчётом от нижней(?) левой точки
    Rect rL, rC;
    private volatile int zone = 0;
    private volatile Mat img;

    public BingusPipeline(boolean isRed) {
        this.isRed = isRed ? 1 : 2;
        rL = new Rect(new Point(xL - offsetX, y - offsetY), new Point(xL + offsetX, y + offsetY));
        rC = new Rect(new Point(xC - offsetX, yC - offsetY), new Point(xC + offsetY, yC + offsetY));
    }

    public void init(Mat firstFrame) {
        inputToCb(firstFrame);
        scanAreaLeft = Ctgt.submat(rL);
        scanAreaCenter = Ctgt.submat(rC);
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
        if (avgL > 120) zone = 0;
        else if (avgC > 120) zone = 1;
        else zone = 2;
        draw(rL, input);
        draw(rC, input);
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
        telemetry.addData("Vals: ", String.format("%1$.2f %2$.2f", avgL, avgC));
        telemetry.update();
        return getAnal();
    }

    private void draw(Rect r, Mat i){
        Imgproc.rectangle(i, r, new Scalar(0, 0, 255), 2);
    }
}

