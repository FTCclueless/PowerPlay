package org.firstinspires.ftc.teamcode.opmodes;

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
    private static final int targetCycles = 5;
    private static final boolean isLeft = true;
    private static final boolean isTop = true;
    private static int parkingNum = 3; // 1, 2, or 3

    // Tile offsets
    private static final int tOffsetx = -12;
    private static final int tOffsety = -8;
    private static OpenCvCamera camera = null;
    private static AprilTagDetectionPipeline atdp = null;

    /* Presets:
     * Tile len: 24
     * Robot total width: 16
     */
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        atdp = new AprilTagDetectionPipeline(
                0.166, // Size of april tag in meters
                // These 4 values are calibration for the C920 webcam (800x448)
                578.272,
                578.272,
                402.145,
                221.506
        );

        camera.setPipeline(atdp);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT); // need to change for phone back camera
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        while (opModeInInit()) {
            telemetry.setMsTransmissionInterval(50);

            boolean detected = false;
            ArrayList<AprilTagDetection> currentDetections = atdp.getLatestDetections();

            if (currentDetections.size() != 0) {

                for (AprilTagDetection tag : currentDetections) {
                    // Add 1 because its 0-2 values not 1-3
                    parkingNum = tag.id + 1;
                    if (parkingNum == 2) {
                        parkingNum = 3;
                    }
                    else if (parkingNum == 3) {
                        parkingNum = 2;
                    }
                    detected = true;
                }

                if (detected) {
                    telemetry.addLine(String.format("Tag of interest is in sight! ID: %d", parkingNum));
                } else {
                    telemetry.addLine("Could not find april tag! :(");
                }
            }

            telemetry.update();
            sleep(20); // FIXME could this be removed?
        }

        int xsign = isTop ? 1 : -1;
        int ysign = isLeft ? 1 : -1;

        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;
        robot.currentState = Robot.STATE.IDLE;

        // 48 - 8 (width / 2) = 40
        Pose2d origin = new Pose2d((48 + tOffsetx) * xsign, (72 + tOffsety) * ysign, (-(Math.PI / 2) * ysign));
        drive.setPoseEstimate(origin);

        TrajectorySequence to = drive.trajectorySequenceBuilder(origin)
                .strafeTo(new Vector2d((48 + tOffsetx) * xsign, (0 + tOffsety) * ysign))
                .strafeTo(new Vector2d((48 + tOffsetx) * xsign, (23 + tOffsety) * ysign))
                .turn(-origin.getHeading())
                .strafeTo(new Vector2d((36 + tOffsetx) * xsign, (23 + tOffsety) * ysign)) // Half tile back
                .build();

        Trajectory cycle1 = drive.trajectoryBuilder(new Pose2d((36 + tOffsetx) * xsign, (23 + tOffsety) * ysign))
                .strafeTo(new Vector2d((72 + tOffsetx) * xsign, (23 + tOffsety) * ysign))
                .build();

        Trajectory cycle2 = drive.trajectoryBuilder(new Pose2d((72 + tOffsetx) * xsign, (23 + tOffsety) * ysign))
                .strafeTo(new Vector2d((36 + tOffsetx) * xsign, (23 + tOffsety) * ysign))
                .build();

        Vector2d parkingPos = null;
        switch (parkingNum) {
            case 1:
                parkingPos = new Vector2d((24 + tOffsetx) * xsign, (23 + tOffsety) * ysign);
                break;
            case 2:
                parkingPos = new Vector2d((48 + tOffsetx) * xsign, (23 + tOffsety) * ysign);
                break;
            case 3:
                parkingPos = new Vector2d((72 + tOffsetx) * xsign, (23 + tOffsety) * ysign);
                break;
        }

        // (72, 24) if even amount of cycles and 36 if not
        Trajectory park = null;
        Pose2d parkingOrigin = new Pose2d((36 + tOffsetx + ((targetCycles & 1) * 36)) * xsign, (23 + tOffsety) * ysign); // where the robot ends at the end of its cycles

        if (parkingOrigin.getX() != parkingOrigin.getX() || parkingOrigin.getY() != parkingOrigin.getY() || parkingOrigin.getHeading() != parkingOrigin.getHeading()) {
            park = drive.trajectoryBuilder(parkingOrigin)
                    .strafeTo(parkingPos)
                    .build();
        }

        waitForStart();

        if (!isStopRequested()) {
            robot.followTrajectorySequence(to);
            for (int i = 0; i < targetCycles; i++) {
                robot.followTrajectory(cycle1);
                if (++i < targetCycles) {
                    robot.followTrajectory(cycle2);
                }
            }
            if (park != null) {
                robot.followTrajectory(park);
            }
        }
    }
}
