package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Robot.STATE.DEPOSIT;
import static org.firstinspires.ftc.teamcode.Robot.STATE.INTAKE_GLOBAL;
import static org.firstinspires.ftc.teamcode.Robot.STATE.SCORING_GLOBAL;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.drive.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(group = "Test")
public class Auto extends LinearOpMode {
    public static final int cycles = 5;
    public static int parkingNum = 0;
    public static final boolean lr = false; // Left : true | Right : false
    public static final boolean tb = false; // Top : true | Bottom : false
    public static final double cycleBack = 6;
    public static final double cycleY = 48; // Turning can give an offset (+ cone location)
    public static OpenCvCamera camera;
    public static final AprilTagDetectionPipeline atdp = new AprilTagDetectionPipeline(
        0.166, // Size of april tag in meters
        // These 4 values are calibration for the C920 webcam (800x448)
        578.272,
        578.272,
        402.145,
        221.506
    );

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.setPipeline(atdp);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT); // need to change for phone back camera
            }

            @Override
            public void onError(int errorCode) {
                Log.e("error with vision", "");
            }
        });

        // Signs
        int xSign = tb ? 1 : -1;
        int ySign = lr ? 1 : -1;

        Pose2d origin = new Pose2d(
            36 * xSign,
            60 * ySign,
            Math.PI / 2 * (!lr ? -1 : 1)
        );

        drive.setPoseEstimate(origin); // FIXME is this needed?
        TrajectorySequence to = drive.trajectorySequenceBuilder(origin)
                // Move forward extra in order to bump away the signal cone
                .strafeTo(new Vector2d(origin.getX(), origin.getY() - (52 * ySign)))
                .strafeTo(new Vector2d(origin.getX(), origin.getY() - (cycleY * ySign))) // because the turning offsets it by a good amount
                .turn(-Math.PI / 2 * xSign * ySign) // FIXME hardcoded (but whatever)
                .strafeTo(new Vector2d(origin.getX() - (cycleBack * xSign), origin.getY() - (cycleY * ySign)))
                .build();

        Pose2d toEnd = to.end(); // Incredible naming!
        TrajectorySequence cycleBack = drive.trajectorySequenceBuilder(toEnd)
                .strafeTo(new Vector2d(toEnd.getX() + (12 + Auto.cycleBack) * xSign, toEnd.getY())) // +3 because it ends at -3
                .build();

        TrajectorySequence cycleForward = drive.trajectorySequenceBuilder(cycleBack.end())
                .strafeTo(new Vector2d(toEnd.getX(), toEnd.getY()))
                .build();

        /** We can use toEnd because that's:
         * Faster
         * The end position
         */
        Trajectory[] park = new Trajectory[]{
            drive.trajectoryBuilder(toEnd).strafeTo(new Vector2d(origin.getX() + (23.5 + (tb ? 0 : 1.5)), origin.getY() - (cycleY * ySign))).build(),
            drive.trajectoryBuilder(toEnd).strafeTo(new Vector2d(origin.getX() - (2 * ySign), origin.getY() - (cycleY * ySign))).build(),
            drive.trajectoryBuilder(toEnd).strafeTo(new Vector2d(origin.getX() - (25), origin.getY() - (cycleY * ySign))).build()
        };

        while (opModeInInit()) {
            telemetry.setMsTransmissionInterval(50);

            boolean detected = false;
            ArrayList<AprilTagDetection> currentDetections = atdp.getLatestDetections();

            if (currentDetections.size() != 0) {
                for (AprilTagDetection tag : currentDetections) {
                    switch (tag.id) {
                        case 2:
                            parkingNum = 1;
                            break;
                        case 1:
                            parkingNum = 2;
                            break;
                        default:
                            parkingNum = tag.id;
                    }
                    detected = true;
                }
            }

            if (detected) {
                telemetry.addLine(String.format("Tag of interest is in sight! ID: %d", parkingNum + 1));
            } else {
                telemetry.addLine("Could not find april tag! :(");
            }

            telemetry.update();
        }

        robot.resetEncoders();
        waitForStart();

        double coneStackAdditionalHeight = 1.38;

        robot.followTrajectorySequence(to, this);

        for (int i = 0; i < cycles; i++) {
            robot.drivetrain.setBreakFollowingThresholds(new Pose2d(2.5, 2.5, Math.toRadians(5)));

            robot.currentState = Robot.STATE.RETRACT;
            robot.startIntakeGlobal(cycleForward.end(),new Pose2d((72-10)*xSign,12*ySign),coneStackAdditionalHeight*(4-i));
            robot.followTrajectorySequence(cycleForward, this);

            while (robot.currentState == INTAKE_GLOBAL) {
                robot.update();
            }

            robot.followTrajectorySequence(cycleBack, this);
            robot.startScoringGlobal(cycleBack.end(), new Pose2d(24*xSign,0),36);
            while (robot.currentState == SCORING_GLOBAL || robot.currentState == DEPOSIT) {
                robot.update();
            }
        }
        robot.drivetrain.setBreakFollowingThresholds(new Pose2d(0.5, 0.5, Math.toRadians(5)));

        robot.followTrajectory(park[parkingNum], this);
    }
}


//        if (!isStopRequested()) {
//            robot.followTrajectorySequence(to, this);
//            for (int i = 0; i < cycles; i++) {
//                robot.followTrajectorySequence(cycleBack, this);
//                robot.followTrajectorySequence(cycleForward, this);
//            }
//            robot.followTrajectory(park[parkingNum], this);
//        }


//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
//import org.firstinspires.ftc.teamcode.modules.drive.roadrunner.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
//import org.openftc.apriltag.AprilTagDetection;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import static org.firstinspires.ftc.teamcode.Robot.STATE.DEPOSIT;
//import static org.firstinspires.ftc.teamcode.Robot.STATE.INTAKE_GLOBAL;
//import static org.firstinspires.ftc.teamcode.Robot.STATE.SCORING_GLOBAL;
//
//import android.util.Log;
//
//import java.util.ArrayList;
//
//@Disabled
//@Autonomous(group = "Test")
//public class Auto extends LinearOpMode {
//    private static final int targetCycles = 5;
//    private static final boolean isLeft = true;
//    private static final boolean isTop = true;
//    private static int parkingNum = 3; // 1, 2, or 3
//
//    // Tile offsets
//    private static final int tOffsetx = -12;
//    private static final int tOffsety = -8;
//    private static OpenCvCamera camera = null;
//    private static AprilTagDetectionPipeline atdp = null;
//
//    /* Presets:
//     * Tile len: 24
//     * Robot total width: 16
//     */
//    @Override
//    public void runOpMode() throws InterruptedException {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        atdp = new AprilTagDetectionPipeline(
//                0.166, // Size of april tag in meters
//                // These 4 values are calibration for the C920 webcam (800x448)
//                578.272,
//                578.272,
//                402.145,
//                221.506
//        );
//
//        camera.setPipeline(atdp);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT); // need to change for phone back camera
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                Log.e("error with vision", "");
//            }
//        });
//
//        int xsign = isTop ? 1 : -1;
//        int ysign = isLeft ? 1 : -1;
//
//        Robot robot = new Robot(hardwareMap);
//        Drivetrain drive = robot.drivetrain;
//
//        // 48 - 8 (width / 2) = 40
//        Pose2d origin = new Pose2d((48 + tOffsetx) * xsign, (72 + tOffsety) * ysign, ((Math.PI / 2) * ysign));
//        drive.setPoseEstimate(origin);
//
//        TrajectorySequence to = drive.trajectorySequenceBuilder(origin)
//                .strafeTo(new Vector2d((48 + tOffsetx) * xsign, (0-tOffsety) * ysign))
//                .strafeTo(new Vector2d((48 + tOffsetx) * xsign, (11) * ysign))
//                .turn(-origin.getHeading())
//                .strafeTo(new Vector2d((37) * xsign, (11) * ysign)) // Half tile back
//                .build();
//
//        Trajectory cycle1 = drive.trajectoryBuilder(new Pose2d((36 + tOffsetx) * xsign, (11) * ysign))
//                .strafeTo(new Vector2d((72 + tOffsetx) * xsign, (11) * ysign))
//                .build();
//
//        Trajectory cycle2 = drive.trajectoryBuilder(new Pose2d((72 + tOffsetx) * xsign, (11) * ysign))
//                .strafeTo(new Vector2d((37) * xsign, (11) * ysign))
//                .build();
//
//        Pose2d parkingOrigin = cycle2.end(); // where the robot ends at the end of its cycles
//        Trajectory[] parks = new Trajectory[] {
//                drive.trajectoryBuilder(parkingOrigin)
//                    .strafeTo(new Vector2d((24 + tOffsetx) * xsign, (11) * ysign))
//                    .build(),
//                drive.trajectoryBuilder(parkingOrigin)
//                    .strafeTo(new Vector2d((48 + tOffsetx) * xsign, (11) * ysign))
//                    .build(),
//                drive.trajectoryBuilder(parkingOrigin)
//                    .strafeTo(new Vector2d((72 + tOffsetx) * xsign, (11) * ysign))
//                    .build()
//        };
//
//        while (opModeInInit()) {
//            telemetry.setMsTransmissionInterval(50);
//
//            boolean detected = false;
//            ArrayList<AprilTagDetection> currentDetections = atdp.getLatestDetections();
//
//            if (currentDetections.size() != 0) {
//
//                for (AprilTagDetection tag : currentDetections) {
//                    // Add 1 because its 0-2 values not 1-3
//                    parkingNum = tag.id + 1;
//                    if (parkingNum == 2) {
//                        parkingNum = 3;
//                    }
//                    else if (parkingNum == 3) {
//                        parkingNum = 2;
//                    }
//                    detected = true;
//                }
//
//                if (detected) {
//                    telemetry.addLine(String.format("Tag of interest is in sight! ID: %d", parkingNum));
//                } else {
//                    telemetry.addLine("Could not find april tag! :(");
//                }
//            }
//
//            telemetry.update();
//        }
//
//        robot.currentState = Robot.STATE.INIT;
//        robot.outtake.resetEncoders();
//
//        waitForStart();
//
//        double coneStackAdditionalHeight = 1.38;
//
//        robot.followTrajectorySequence(to, this);
//
//        for (int i = 0; i < targetCycles; i++) {
//            robot.drivetrain.setBreakFollowingThresholds(new Pose2d(2.5, 2.5, Math.toRadians(7)));
//
//            robot.currentState = Robot.STATE.RETRACT;
//            robot.startIntakeGlobal(cycle1.end(),new Pose2d((72-4)*xsign,12*ysign),coneStackAdditionalHeight*(4-i));
//
//            robot.followTrajectory(cycle1, this);
//            while (robot.currentState == INTAKE_GLOBAL) {
//                robot.update();
//            }
//
//            robot.startScoringGlobal(cycle2.end(),new Pose2d(24*xsign,0),36);
//            robot.followTrajectory(cycle2, this);
//            while (robot.currentState == SCORING_GLOBAL || robot.currentState == DEPOSIT) {
//                robot.update();
//            }
//        }
//        robot.drivetrain.setBreakFollowingThresholds(new Pose2d(0.5, 0.5, Math.toRadians(5)));
//
//        robot.followTrajectory(parks[parkingNum], this);
//
//        drive.setMotorPowers(0,0,0,0);
//    }
//}