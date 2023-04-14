package org.firstinspires.ftc.teamcode.modules.vision;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.java_websocket.framing.PongFrame;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class VisionPipeline extends OpenCvPipeline {
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

    Mat cbMat = new Mat();
    Mat deNoiseMat = new Mat();
    Mat averageMat = new Mat();

    double[] pole = new double[2];

    static final int YCRCB_CHANNEL_IDX = 2;

    static final Point REGION1_TOP_LEFT_ANCHOR_POINT = new Point(30,0); // change x coordinate to change height where we take 4 rows
    static final int REGION_WIDTH = 4;
    static final int REGION_HEIGHT = 240; // 1080

    Point region1_pointA = new Point(
            REGION1_TOP_LEFT_ANCHOR_POINT.x,
            REGION1_TOP_LEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOP_LEFT_ANCHOR_POINT.x - REGION_WIDTH,
            REGION1_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    @Override
    public Mat processFrame(Mat input)
    {
        // convert to CB color space
        Imgproc.cvtColor(input, cbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(cbMat, cbMat, YCRCB_CHANNEL_IDX);

        // de-noise image
        noiseReduction(cbMat, deNoiseMat);

        // average rows
        Imgproc.resize(deNoiseMat, averageMat, new Size(1,240));

        // search for blobs/patches
        pole = findLongestLength(averageMat);

        // get center pixel value of patch
        // get width of patch
        // perform transformations to get angle + distance away

        Imgproc.rectangle(
                input,
                region1_pointA,
                region1_pointB,
                new Scalar(0, 255, 0), 1);

        Imgproc.rectangle(
                input,
                new Point(region1_pointA.x, pole[0]),
                new Point(region1_pointB.x, pole[1]),
                new Scalar(255, 0, 0), 1);

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

    int longestLength = 0;
    int currentLength = 0;
    int currentStart = 0;
    int longestStart = 0;

    private double[] findLongestLength(Mat input) {
        for (int i = 0; i < input.rows(); i++) {
            double currentPixel = input.get(i, 0)[0];
            if (currentPixel >= min && currentPixel <= max) {
                currentLength += 1;
            } else {
                currentLength = 0;
                currentStart = i+1;
            }

            if (currentLength > longestLength) {
                longestLength = currentLength;
                longestStart = currentStart;
            }

            Log.e("currentPixel", currentPixel + "");
        }
        Log.e("NEW LINE","------------");

        return new double[] {(double) longestStart, (double) longestLength};
    }
}
