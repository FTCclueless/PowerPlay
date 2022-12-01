package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfRect;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.CascadeClassifier;
import org.openftc.easyopencv.OpenCvPipeline;


import java.util.ArrayList;
import java.util.List;

public class OpenCVObjectDetector extends MyOpenCvPipeline {
    public enum SkystoneLocation {
        LEFT,
        RIGHT,
        NONE
    }

    private String TAG = "OpenCVObjectDetector";
    static public String ftcDirPath = "/sdcard/FIRST/";
    private String saveImgFileName = ftcDirPath + "capture.jpg";
    private boolean doneImageCapture = false;
    private int width; // width of the image
    public SkystoneLocation location;

    public void setTelemetry(Telemetry tele) {
        telemetry = tele;
    }


    /**
     * @param width The width of the image (check your camera)
     */
    public OpenCVObjectDetector(int width) {
        this.width = width;
    }

    @Override
    public Mat processFrame(Mat input) {
        // "Mat" stands for matrix, which is basically the image that the detector will process
        // the input matrix is the image coming from the camera
        // the function will return a matrix to be drawn on your phone's screen

        // The detector detects regular stones. The camera fits two stones.
        // If it finds one regular stone then the other must be the skystone.
        // If both are regular stones, it returns NONE to tell the robot to keep looking
        RobotLogger.dd(TAG, input.rows() + " " + input.cols() + " imaged saved to " + saveImgFileName);

        if (!doneImageCapture) {
            Imgcodecs imageCodecs = new Imgcodecs();
            imageCodecs.imwrite(saveImgFileName, input);
            doneImageCapture = true;
        }

        /*  test!!!
         * Draw a simple box around the middle 1/2 of the entire frame

        Imgproc.rectangle(
                input,
                new Point(
                        input.cols()/4,
                        input.rows()/4),
                new Point(
                        input.cols()*(3f/4f),
                        input.rows()*(3f/4f)),
                new Scalar(0, 255, 0), 4);
        */

        // Make a working copy of the input matrix in HSV
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // if something is wrong, we assume there's no skystone
        if (mat.empty()) {
            location = SkystoneLocation.NONE;
            return input;
        }


        // We create a HSV range for yellow to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
        Scalar lowHSV = new Scalar(20, 100, 100); // lower bound HSV for yellow
        Scalar highHSV = new Scalar(30, 255, 255); // higher bound HSV for yellow
        Mat thresh = new Mat();

        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(mat, lowHSV, highHSV, thresh);

        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis
        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 300);


        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.
        // We then find the bounding rectangles of those contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxValArea = 0;
        int maxValIdx = 0;
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];

        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));

            //find max area
            double contourArea = Imgproc.contourArea(contours.get(i));
            if (contourArea > maxValArea) {
                maxValArea = contourArea;
                maxValIdx = i;
            }

        }

        Point centerOut = new Point(0, 0);
        float[] radiusOut = new float[1];

        //convert matOfPoint to matOfPoint2f
        if (contours.size() == 0)
        {
            telemetry.addData("NO contour!", "empty");
            return input;
        }
        MatOfPoint temp = contours.get(maxValIdx);
        MatOfPoint2f temp2f = new MatOfPoint2f(temp.toArray());

        Imgproc.minEnclosingCircle(temp2f, centerOut, radiusOut);

        // two return values are area = maxVal and center = centerOut


        // Iterate and check whether the bounding boxes
        // cover left and/or right side of the image
        double left_x = 0.25 * width;
        double right_x = 0.75 * width;
        boolean everOther = true;

        boolean left = false; // true if regular stone found on the left side
        boolean right = false; // "" "" on the right side
        for (int i = 0; i != boundRect.length; i++) {
            if (boundRect[i].x < left_x)
                left = true;
            if (boundRect[i].x + boundRect[i].width > right_x)
                right = true;


            // draw red bounding rectangles on mat
            // the mat has been converted to HSV so we need to use HSV as well
            if (everOther = (true)) {
                Imgproc.rectangle(mat, boundRect[i], new Scalar(0.5, 76.9, 89.8));
            }}


            // if there is no yellow regions on a side
            // that side should be a Skystone

        /*if (!left) location = SkystoneLocation.LEFT;
        else if (!right) location = SkystoneLocation.RIGHT;
            // if both are true, then there's no Skystone in front.
            // since our team's camera can only detect two at a time
            // we will need to scan the next 2 stones
        else location = SkystoneLocation.NONE; */

            // convert back for viewing
            Mat output = new Mat();
            Imgproc.cvtColor(mat, output, Imgproc.COLOR_HSV2RGB);
            Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 0));
        RobotLogger.dd(TAG, "Max Contour Area: " + maxValArea + " " + SleeveColors.values()[maxValIdx]);
        telemetry.addData("Max Contour Area: " + maxValArea, "%s %f", SleeveColors.values()[maxValIdx]);
        RobotLogger.dd(TAG, "Center of Contour: " + centerOut.x + " " + centerOut.y);
        telemetry.addData("center of contour: ", "%s %f", centerOut.x, " ", "%s %f", centerOut.y);

            return output; // return the mat with rectangles drawn



            }

        }

    /*public SkystoneLocation getLocation() {
        return this.location;
    } */


    class CustomizedObjDetect {
        public void doWork(Mat input) {
            Imgcodecs imageCodecs = new Imgcodecs();
            CascadeClassifier face_cascade = new CascadeClassifier(OpenCVObjectDetector.ftcDirPath + "haarcascade_frontalface_default.xml");
            Mat gray = new Mat();
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
            //    # Detect the faces
            //Detecting the face in the snap
            MatOfRect faceDetections = new MatOfRect();
            face_cascade.detectMultiScale(gray, faceDetections, 1.1);
            int i = 0;
            for (Rect rect : faceDetections.toArray()) {
                Imgproc.rectangle(input, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(0, 255, 0));
                imageCodecs.imwrite(OpenCVObjectDetector.ftcDirPath + String.valueOf(i) + ".jpg", input);
                i = i + 1;
            }
            //# Stop if escape key is pressed;
        }
    }
