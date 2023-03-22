package org.firstinspires.ftc.teamcode.util;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;

import org.firstinspires.ftc.teamcode.modules.drive.basilisk.Spline;

import java.util.List;

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */
public class DashboardUtil {
    private static final double ROBOT_RADIUS = 9; // in

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

    public static void drawSampledPath(Canvas canvas, Spline spline) {
        int samples = spline.points.size();
        double[] xPoints = new double[samples];
        double[] yPoints = new double[samples];
        for (int i = 0; i < samples; i++) {
            Pose2d pose = spline.points.get(i);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);

        Log.e("samples", samples + "");
    }

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS);
        Pose2d v = new Pose2d(Math.cos(pose.heading)*ROBOT_RADIUS, Math.sin(pose.heading)*ROBOT_RADIUS);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }
}
