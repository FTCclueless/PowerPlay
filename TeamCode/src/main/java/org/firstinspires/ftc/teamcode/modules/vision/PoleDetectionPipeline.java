package org.firstinspires.ftc.teamcode.modules.vision;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.outtake.Outtake;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class PoleDetectionPipeline extends OpenCvPipeline {
    /*
     * Points which actually define the sample region rectangles, derived from above values
     *
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     *
     */

    Mat croppedMat = new Mat();
    Mat cbMat = new Mat();
    Mat deNoiseMat = new Mat();
    Mat averageMat = new Mat();

    double[] pole = new double[2];

    static final int YCRCB_CHANNEL_IDX = 2;

    static final Point REGION1_TOP_LEFT_ANCHOR_POINT = new Point(40,0); // change x coordinate to change height where we take 4 rows
    static final int REGION_WIDTH = 4;
    static final int REGION_HEIGHT = 240; // 1080

    Point region1_pointA = new Point(
            REGION1_TOP_LEFT_ANCHOR_POINT.x,
            REGION1_TOP_LEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOP_LEFT_ANCHOR_POINT.x - REGION_WIDTH,
            REGION1_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // math variables
    double verticalResolution = 240;
    double fov = Math.toRadians(47.3); // radians
    double poleWidthInInches = 1;

    double distanceFromPole;
    double angleFromPole;

    double cameraXOffset = 0.34;
    double cameraYOffset = -8.27;
    double cameraAngleOffset = Math.toRadians(10);

    ArrayList<Double> distanceData = new ArrayList<Double>();
    double averageDistance = 0;

    ArrayList<Double> angleData = new ArrayList<Double>();
    double averageAngle = 0;

    public Pose2d globalPolePose = new Pose2d(0,0);

    Robot robot;
    Drivetrain drivetrain;
    Outtake outtake;
    Turret turret;

    public PoleDetectionPipeline(Robot robot, Drivetrain drivetrain, Outtake outtake) {
        this.robot = robot;
        this.drivetrain = drivetrain;
        this.outtake = outtake;
        this.turret = outtake.turret;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        // crop in
        croppedMat = input.submat(new Rect(region1_pointA, region1_pointB));

        // convert to CB color space
        Imgproc.cvtColor(croppedMat, cbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(cbMat, cbMat, YCRCB_CHANNEL_IDX);

        // de-noise image
        noiseReduction(cbMat, deNoiseMat);

        // average rows
        Imgproc.resize(deNoiseMat, averageMat, new Size(1,240));

        // search for blobs/patches
        pole = findLongestLength(averageMat);

        // perform transformations to get pole's distance away
        double poleWidthInPixels = pole[1];

        if (poleWidthInPixels != 0) {
            distanceFromPole = (poleWidthInInches/2)/(Math.tan(((poleWidthInPixels)/verticalResolution)*fov));

            double distanceSum = 0;

            distanceData.add(distanceFromPole);
            if (distanceData.size() > 10) {
                distanceData.remove(0);
            }
            for (double a : distanceData) {
                distanceSum += a;
            }

            averageDistance = distanceSum / distanceData.size();

            // perform transformations to get pole's angle away
            double polePositionInPixels = pole[0] + (pole[1]/2);
            angleFromPole = fov * (0.5 - (polePositionInPixels/verticalResolution)) * -1;

            double angleSum = 0;

            angleData.add(angleFromPole);
            if (angleData.size() > 10) {
                angleData.remove(0);
            }
            for (double a : angleData) {
                angleSum += a;
            }

            averageAngle = (angleSum / angleData.size()) + cameraAngleOffset;

            // get global pose of pole

            // first step (gets relative x,y coordinates of pole from the center of turret)
            Pose2d relativePolePoseToTurret = new Pose2d(cameraXOffset + Math.cos(averageAngle)*averageDistance,cameraYOffset + Math.sin(averageAngle)*averageDistance);

            // second step (gets relative x,y coordinates of pole from the center of robot, taking into account the rotation of the turret)
            double x = outtake.turretXOffset + relativePolePoseToTurret.x*Math.cos(turret.currentTurretAngle) - relativePolePoseToTurret.y*Math.sin(turret.currentTurretAngle);
            double y = outtake.turretYOffset + relativePolePoseToTurret.y*Math.cos(turret.currentTurretAngle) + relativePolePoseToTurret.x*Math.sin(turret.currentTurretAngle);
            Pose2d relativePolePoseToRobot = new Pose2d(x, y);

            // third step (gets global x,y coordinates of pole, taking into account the rotation of the robot)
            Pose2d robotPose = drivetrain.getPoseEstimate();
            x = robotPose.getX() + relativePolePoseToRobot.x*Math.cos(robotPose.getHeading()) - relativePolePoseToRobot.y*Math.sin(robotPose.getHeading());
            y = robotPose.getY() + relativePolePoseToRobot.y*Math.cos(robotPose.getHeading()) + relativePolePoseToRobot.x*Math.sin(robotPose.getHeading());

            globalPolePose = new Pose2d(Math.abs(x) < 72 ? x : 0, Math.abs(y) < 72 ? y : 0);;

            // draw cropped area
            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    new Scalar(0, 255, 0), 1);

            // pole
            Imgproc.rectangle(
                    input,
                    new Point(region1_pointA.x, pole[0]),
                    new Point(region1_pointB.x, pole[0]+pole[1]),
                    new Scalar(255, 0, 0), 1);

            Log.e("distanceFromPole", distanceFromPole + "");
            Log.e("angleFromPole", Math.toDegrees(angleFromPole) + "");

            Log.e("distanceAverage", averageDistance + "");
            Log.e("angleAverage", Math.toDegrees(averageAngle) + "");
        }

        return input;
    }

    /*
     * The elements we use for noise reduction
     */
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(4, 4));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));

    private void noiseReduction(Mat input, Mat output)
    {
        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);

        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    public double min = 0;
    public double max = 100;

    public double mean = 119.1964286;
    public double standard_deviation = 11.78304573;

    private double[] findLongestLength(Mat input) {

        double calculated_mean = 0.0;
        double calculated_standard_deviation = 0.0;

        double longestLength = 0;
        double currentLength = 0;
        double currentStart = 0;
        double longestStart = 0;

        for (int i = 0; i < input.rows(); i++) {
            double currentPixel = input.get(i, 0)[0];

            calculated_mean += currentPixel;
            calculated_standard_deviation += Math.pow((currentPixel - mean), 2);

            max = mean - (standard_deviation * 3);

            if (currentPixel >= min && currentPixel <= max) {
                currentLength += 1;

                if (currentLength > longestLength) {
                    longestLength = currentLength;
                    longestStart = currentStart;
                }
            } else {
                currentLength = 0;
                currentStart = i+1;
            }
        }

        calculated_mean = calculated_mean / verticalResolution;
        calculated_standard_deviation = Math.sqrt(calculated_standard_deviation / verticalResolution);

        Log.e("calculated_standard_deviation", calculated_standard_deviation + "");

        mean = calculated_mean;
        standard_deviation = Math.max(calculated_standard_deviation, 7); // basically if there isn't anything statistically significant it won't do anything

        return new double[] { longestStart, longestLength};
    }
}
