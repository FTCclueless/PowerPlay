package org.firstinspires.ftc.teamcode.vision;

import static org.opencv.core.Core.inRange;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

abstract class MyOpenCvPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    public void setTelemetry(Telemetry tele) {
        telemetry = tele;
    }
    public ArrayList<AprilTagDetection> getLatestDetections() {
        return null;
    }
}
//https://github.com/superarash1/HHH_FTC_PowerPlay_2022-2033/blob/main/Team8109_Rise/src/main/java/org/firstinspires/ftc/team8109_Rise/Hardware/Sensors/Camera/OpenCV/VisionPipelines/ConeTracker.java
public class ConeTracker extends MyOpenCvPipeline {
    private String TAG = "ConeTracker";
    static final Scalar GREEN = new Scalar(0, 255, 0);

    boolean RED = true;
    boolean BLUE = true;

    public int redContourCount = 0;
    public int blueContourCount = 0;

    public List<Rect> redRect;
    public List<Rect> blueRect;

    public Rect RedRect;
    public Rect BlueRect;

    public List<MatOfPoint> redContours;
    public List<MatOfPoint> blueContours;

    public MatOfPoint biggestRedContour;
    public MatOfPoint biggestBlueContour;

    public ConeTracker() {
        redContours = new ArrayList<MatOfPoint>();
        redRect = new ArrayList<Rect>();
        RedRect = new Rect();
        biggestRedContour = new MatOfPoint();

        blueContours = new ArrayList<MatOfPoint>();
        blueRect = new ArrayList<Rect>();
        BlueRect = new Rect();
        biggestBlueContour = new MatOfPoint();

        //this.telemetry = telemetry;
    }
    public void setTelemetry(Telemetry tele) {
        telemetry = tele;
    }

    // Filters the contours to be greater than a specific area in order to be tracked
    public boolean filterContours(MatOfPoint contour) {
        return Imgproc.contourArea(contour) > 50;
    }

    // Red masking thresholding values:
    Scalar lowRed = new Scalar(0, 160, 0); //10, 100, 50
    Scalar highRed = new Scalar(255, 255, 255); //35, 255, 255

    // Blue masking thresholding values:
    Scalar lowBlue = new Scalar(0, 0, 140); //10, 100, 50
    Scalar highBlue = new Scalar(175, 255, 255); //35, 255, 255

    // Mat object for the red and blue mask
    Mat maskRed = new Mat();
    Mat maskBlue = new Mat();

    // Mat object for YCrCb color space
    Mat YCrCb = new Mat();

    // Kernel size for blurring
    Size kSize = new Size(5, 5);
    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size((2 * 2) + 1, (2 * 2) + 1));

    @Override
    public Mat processFrame(Mat input) {
        RobotLogger.dd(TAG, "received one frame");
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Imgproc.erode(YCrCb, YCrCb, kernel);

        if (RED) {
            // Finds the pixels within the thresholds and puts them in the mat object "maskRed"
            inRange(YCrCb, lowRed, highRed, maskRed);

            // Clears the arraylists
            redContours.clear();
            redRect.clear();

            // Finds the contours and draws them on the screen
            Imgproc.findContours(maskRed, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, redContours, -1, GREEN); //input

            // Iterates through each contour
            for (int i = 0; i < redContours.size(); i++) {

                // Filters out contours with an area less than 50 (defined in the filter contours method)
                if (filterContours(redContours.get(i))) {
                    biggestRedContour = Collections.max(redContours, (t0, t1) -> {
                        return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
                    });

                    // Creates a bounding rect around each contourand the draws it
                    RedRect = Imgproc.boundingRect(biggestRedContour);

                    Imgproc.rectangle(input, RedRect, GREEN, 2);
                }
            }

            // Displays the position of the center of each bounding rect (rect.x/y returns the top left position)
            telemetry.addData("Red Contour ", "%7d,%7d", RedRect.x + (RedRect.width/2), RedRect.y + (RedRect.height/2));

            maskRed.release();
        }

        if (BLUE) {
            // Finds the pixels within the thresholds and puts them in the mat object "maskBlue"
            inRange(YCrCb, lowBlue, highBlue, maskBlue);

            // Clears the arraylists
            blueContours.clear();
            blueRect.clear();

            // Finds the contours and draws them on the screen
            Imgproc.findContours(maskBlue, blueContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, blueContours, -1, GREEN); //input

            // Iterates through each contour
            for (int i = 0; i < blueContours.size(); i++) {
                // Filters out contours with an area less than 50 (defined in the filter contours method)
                if (filterContours(blueContours.get(i))) {
                    biggestBlueContour = Collections.max(blueContours, (t0, t1) -> {
                        return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
                    });

                    // Creates a bounding rect around each contourand the draws it
                    BlueRect = Imgproc.boundingRect(biggestBlueContour);
                    Imgproc.rectangle(input, BlueRect, GREEN, 2);
                }
            }

            // Displays the position of the center of each bounding rect (rect.x/y returns the top left position)
            telemetry.addData("Blue Contour ", "%7d,%7d", BlueRect.x + (BlueRect.width/2), BlueRect.y + (BlueRect.height/2));

            maskBlue.release();
        }

        redContourCount = 0;
        blueContourCount = 0;

        YCrCb.release();

        //TODO: move this when actually using code (this is just for EasyOpenCV sim)
        telemetry.update();

        return input;
    }
}