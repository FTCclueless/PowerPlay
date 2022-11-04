package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.sql.Array;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static org.opencv.core.Core.inRange;
import static org.opencv.core.Core.max;

class ScalarLowHighRanges {
    Scalar low, high;
    ScalarLowHighRanges(Scalar low_, Scalar high_) {
        low = low_;
        high = high_;
    }
}

enum SleeveColors {
    GREEN, PINK, PURPLE
}

public class SleeveDetectCVPipeline  extends MyOpenCvPipeline {
    private String TAG = "SleeveDetectCVPipeline";

    private List<MatOfPoint> allContours;
    private MatOfPoint largestContour;

    private ScalarLowHighRanges[] scalarLowHighRanges;

    public SleeveDetectCVPipeline() {
        allContours = new ArrayList<MatOfPoint>();
        largestContour = null;
        scalarLowHighRanges = new ScalarLowHighRanges[SleeveColors.values().length];
        scalarLowHighRanges[SleeveColors.GREEN.ordinal()] = new ScalarLowHighRanges(new Scalar(56, 35, 0), new Scalar(127, 255, 255));
        scalarLowHighRanges[SleeveColors.PINK.ordinal()] = new ScalarLowHighRanges(new Scalar(141, 22, 0), new Scalar(179, 255, 255));
        scalarLowHighRanges[SleeveColors.PURPLE.ordinal()] = new ScalarLowHighRanges(new Scalar(69, 64, 0), new Scalar(140, 255, 255));
    }
    public void setTelemetry(Telemetry tele) {
        telemetry = tele;
    }

    // Kernel size for blurring
    Size kSize = new Size(5, 5);
    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size((2 * 2) + 1, (2 * 2) + 1));

    Mat maskedImg = new Mat();
    Mat imgHSV = new Mat();
    Mat bluredImg = new Mat();
    Mat bitwisedImg = new Mat();
    Mat grayImg = new Mat();
    Mat erodedImg = new Mat();

    int imgCount = 0;
    Imgcodecs imageCodecs = new Imgcodecs();
    boolean enableDebugFileWriting = false;

    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        MatOfPoint biggest = null;
        if (contours.size()>0) {
            double maxVal = 0;
            int maxValIdx = 0;
            for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++) {
                double contourArea = Imgproc.contourArea(contours.get(contourIdx));
                if (maxVal < contourArea) {
                    maxVal = contourArea;
                    maxValIdx = contourIdx;
                }
            }
            biggest = contours.get(maxValIdx);
        }
        return biggest;
    }

    private MatOfPoint cvProcessingPerColor(Mat imgHSV, int idx) { // don't overwrite imgHSV
        inRange(imgHSV, scalarLowHighRanges[idx].low, scalarLowHighRanges[idx].high, maskedImg);
        Imgproc.erode(maskedImg, erodedImg, kernel);
        Imgproc.dilate(erodedImg, grayImg, kernel);

        //Core.bitwise_and(maskedImg, maskedImg, bitwisedImg, maskedImg);
        //Imgproc.cvtColor(bitwisedImg, grayImg, Imgproc.COLOR_BGR2GRAY);

        allContours.clear();
        Imgproc.findContours(grayImg, allContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        return findLargestContour(allContours);
    }

    @Override
    public Mat processFrame(Mat input) {
        RobotLogger.dd(TAG, "received one frame");
        if ((enableDebugFileWriting == true) && (imgCount % 30 == 0)) {
            String file_path_name = "/sdcard/FIRST/sleeve" + String.valueOf(imgCount/30 % 20) + ".jpg";
            imageCodecs.imwrite(file_path_name, input);
            RobotLogger.dd(TAG, file_path_name + " saved");
        }
        imgCount ++;

        Imgproc.GaussianBlur(input, bluredImg, new Size(11, 11), 0);
        Imgproc.cvtColor(bluredImg, imgHSV, Imgproc.COLOR_RGB2HSV);

        int maxAt = 0;
        double biggestArea = 0.0;
        int numOfColors = SleeveColors.values().length;
        for (int i = 0; i < numOfColors; i ++) {
            MatOfPoint colorContour = cvProcessingPerColor(imgHSV, i);
            if (colorContour != null) {
                double area = Imgproc.contourArea(colorContour);
                RobotLogger.dd(TAG, "contour area for color " + SleeveColors.values()[i] + " " + area);
                if (area > biggestArea) {
                    maxAt = i;
                    biggestArea = area;
                    largestContour = colorContour;
                }
            }
            else {
                RobotLogger.dd(TAG, "No contour for color " + SleeveColors.values()[i]);
            }
        }

        if (biggestArea > 2000) { // to be decided
            List<MatOfPoint> contours = new ArrayList<>();
            assert (largestContour != null);
            contours.add(largestContour);
            Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 0), 5); //input
            RobotLogger.dd(TAG, "max contour area: " + biggestArea + " " + SleeveColors.values()[maxAt]);
            telemetry.addData("max contour area: ", "%s %f", SleeveColors.values()[maxAt], biggestArea);
        }
        else {
            RobotLogger.dd(TAG, "no significant contour detected for color " + SleeveColors.values()[maxAt]);
            telemetry.addData("no significant contour detected for color ", "%s", SleeveColors.values()[maxAt]);
        }
        telemetry.update();

        maskedImg.release();
        imgHSV.release();
        bluredImg.release();
        bitwisedImg.release();
        grayImg.release();
        erodedImg.release();
        return input;
    }
}