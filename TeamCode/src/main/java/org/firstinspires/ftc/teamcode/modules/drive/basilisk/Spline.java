package org.firstinspires.ftc.teamcode.modules.drive.basilisk;

import org.firstinspires.ftc.teamcode.util.Pose2d;

import java.util.ArrayList;

public class Spline {
    public ArrayList<Pose2d> points;

    public Spline (Pose2d startPose) {
        points.add(startPose);
    }

    public Spline addPoint (Pose2d newPoint) {
        // https://www.desmos.com/calculator/yi3jovk0hp

        double[] xCoefficents = new double[4];
        double[] yCoefficents = new double[4];

        Pose2d lastPoint = points.get(points.size()-1); // when you add a new spline the last point becomes the starting point for the new spline

        double arbitraryVelocity = 1.25*Math.sqrt(Math.pow((lastPoint.x - newPoint.x),2) + Math.pow((lastPoint.y - newPoint.y),2));

        xCoefficents[0] = lastPoint.x;
        xCoefficents[1] = arbitraryVelocity * Math.cos(lastPoint.heading);
        xCoefficents[2] = 3*newPoint.x - arbitraryVelocity*Math.cos(newPoint.heading) - 2*xCoefficents[1] - 3*xCoefficents[0];
        xCoefficents[3] = newPoint.x - xCoefficents[0] - xCoefficents[1] - xCoefficents[2];

        yCoefficents[0] = lastPoint.y;
        yCoefficents[1] = arbitraryVelocity * Math.sin(lastPoint.heading);
        yCoefficents[2] = 3*newPoint.y - arbitraryVelocity*Math.sin(newPoint.heading) - 2*yCoefficents[1] - 3*yCoefficents[0];
        yCoefficents[3] = newPoint.y - yCoefficents[0] - yCoefficents[1] - yCoefficents[2];

        Pose2d point = newPoint;

        for (double t2 = 0.0; t2 < 1; t2+=0.001) {
            point.x = xCoefficents[0] + xCoefficents[1]*t2 + xCoefficents[2]*t2*t2 + xCoefficents[3]*t2*t2*t2;
            point.y = yCoefficents[0] + yCoefficents[1]*t2 + yCoefficents[2]*t2*t2 + yCoefficents[3]*t2*t2*t2;

            if(lastPoint.getDistance(point) > 2.0) {
                // gets the velocity because the derivative of position = velocity
                double velX = xCoefficents[1] + 2.0*xCoefficents[2]*t2 + 3.0*xCoefficents[3]*t2*t2;
                double velY = yCoefficents[1] + 2.0*yCoefficents[2]*t2 + 3.0*yCoefficents[3]*t2*t2;

                // heading is equal to the inverse tangent of velX and velY because velX and velY have a magnitude and a direction and soh cah toa
                point.heading = Math.atan2(velY,velX);

                points.add(point);
                lastPoint = point;
            }
        }

        points.add(newPoint);

        return this;
    }

    public double minimumRobotDistanceFromPoint = 8.0;
    double minimumRobotThresholdFromEndPoint = 0.5;

    public Pose2d getErrorFromNextPoint(Pose2d currentRobotPose) {
        if(points.size() == 0){
            return null;
        }

        // loops through all of the points and removes any points that are within the robotsMinimumDistanceFromPoint. When we remove a point the nextPoint becomes the 0th index so we don't increment anything
        while(points.size() > 1 && points.get(0).getDistance(currentRobotPose) < minimumRobotDistanceFromPoint) {
            points.remove(0);
        }

        // checking if we have finished the spline
        if (points.get(0).getDistance(currentRobotPose) < minimumRobotThresholdFromEndPoint) {
            points.remove(0);
            return null;
        }

        // global error to relative error (https://drive.google.com/file/d/1bqHU0ZHKN2yaxgf4M6FBV36Sv0TX0C1g/view?usp=sharing)
        Pose2d globalError = new Pose2d(points.get(0).x- currentRobotPose.x, points.get(0).y - currentRobotPose.y);
        Pose2d relativeError = new Pose2d(
                globalError.x*Math.cos(currentRobotPose.heading) + globalError.y*Math.sin(currentRobotPose.heading),
                globalError.y*Math.cos(currentRobotPose.heading) - globalError.x*Math.sin(currentRobotPose.heading),
                points.get(0).heading-currentRobotPose.heading);

        return relativeError;
    }
}
