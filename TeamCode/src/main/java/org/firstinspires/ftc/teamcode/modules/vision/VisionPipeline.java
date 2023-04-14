package org.firstinspires.ftc.teamcode.modules.vision;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
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

    Mat region = new Mat();
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();

    Mat average = new Mat();

    static final Point REGION1_TOP_LEFT_ANCHOR_POINT = new Point(1920,0); // change x coordinate to change height where we take 4 rows
    static final int REGION_WIDTH = 4;
    static final int REGION_HEIGHT = 1080;

    Point region1_pointA = new Point(
            REGION1_TOP_LEFT_ANCHOR_POINT.x,
            REGION1_TOP_LEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOP_LEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    public static long timeToProcess;
    long start;

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    @Override
    public void init(Mat firstFrame)
    {
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
        inputToCb(firstFrame);

        // grab 4 rows
        region = Cb.submat(new Rect(region1_pointA, region1_pointB));
    }

    @Override
    public Mat processFrame(Mat input)
    {
        start = System.currentTimeMillis();
        // convert to CB color space
        inputToCb(input);

        // average rows
        Imgproc.resize(input, average, new Size(1,1080));
        Log.e("---------------------------ONE FRAME---------------------------", average.toString());
        Log.e("Average:", average.toString());

        // get average yellowness of image

        // search for blobs/patches

        // get center pixel value of patch
        // get width of patch
        // perform transformations to get angle + distance away

        timeToProcess = System.currentTimeMillis() - start;
        return input;
    }
}
