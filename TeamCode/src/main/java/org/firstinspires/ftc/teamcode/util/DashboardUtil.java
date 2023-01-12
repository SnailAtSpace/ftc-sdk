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
    private static final double DEFAULT_RESOLUTION = 30; // distance units; presumed inches
    private static final double width = 380, length = 330, diag = Math.hypot(width, length);
    private static final double hWidth = width/2, hLength = length/2,hDiag=diag/2, fieldHalf = 1790.7;
    private static final double angleDev = Math.asin(hWidth/hDiag);


    public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose2d pose = poseHistory.get(i);
            xPoints[i] = pose.getX()/25.4;
            yPoints[i] = pose.getY()/25.4;
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
            xPoints[i] = pose.getX()/25.4;
            yPoints[i] = pose.getY()/25.4;
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path) {
        drawSampledPath(canvas, path, DEFAULT_RESOLUTION);
    }

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX()/25.4, pose.getY()/25.4, hDiag/25.4);
        Vector2d v = pose.headingVec().times(hDiag/25.4);
        canvas.strokePolygon(
                new double[]{pose.getX()/25.4+v.rotated(angleDev).getX(),
                                pose.getX()/25.4+v.rotated(-angleDev).getX(),
                                pose.getX()/25.4-v.rotated(angleDev).getX(),
                                pose.getX()/25.4-v.rotated(-angleDev).getX()},
                new double[]{pose.getY()/25.4+v.rotated(angleDev).getY(),
                                pose.getY()/25.4+v.rotated(-angleDev).getY(),
                                pose.getY()/25.4-v.rotated(angleDev).getY(),
                                pose.getY()/25.4-v.rotated(-angleDev).getY()});
        v = pose.headingVec().times(hLength);
        double x1 = pose.getX(), y1 = pose.getY();
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1/25.4, y1/25.4, x2/25.4, y2/25.4);
    }
}
