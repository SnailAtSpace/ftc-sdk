package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;

import java.util.List;

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */
public class DashboardUtil {
    private static final double DEFAULT_RESOLUTION = 2.0; // distance units; presumed inches
    private static final double ROBOT_RADIUS = 9; // in
    private static final double width = 330.29/25.4, length = 380.78/25.4, diag = 503.8783666/25.4;
    private static final double hWidth = width/2, hLength = length/2,hDiag=diag/2, fieldHalf = 70.5;
    private static final double angleDev = Math.asin(hWidth/hDiag);


    public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose2d pose = poseHistory.get(i);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path, double resolution) {
        int samples = (int) Math.ceil(path.length() / resolution);
        double[] xPoints = new double[samples];
        double[] yPoints = new double[samples];
        double dx = path.length() / (samples - 1);
        for (int i = 0; i < samples; i++) {
            double displacement = i * dx;
            Pose2d pose = path.get(displacement);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path) {
        drawSampledPath(canvas, path, DEFAULT_RESOLUTION);
    }

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), hDiag);
        Vector2d v = pose.headingVec().times(hDiag);
        canvas.strokePolygon(
                new double[]{pose.getX()+v.rotated(angleDev).getX(),
                                pose.getX()+v.rotated(-angleDev).getX(),
                                pose.getX()-v.rotated(angleDev).getX(),
                                pose.getX()-v.rotated(-angleDev).getX()},
                new double[]{pose.getY()+v.rotated(angleDev).getY(),
                                pose.getY()+v.rotated(-angleDev).getY(),
                                pose.getY()-v.rotated(angleDev).getY(),
                                pose.getY()-v.rotated(-angleDev).getY()});
        v = pose.headingVec().times(hLength);
        double x1 = pose.getX(), y1 = pose.getY();
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }
}
