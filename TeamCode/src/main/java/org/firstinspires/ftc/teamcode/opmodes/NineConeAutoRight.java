package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Robot.STATE.DEPOSIT_AUTO;
import static org.firstinspires.ftc.teamcode.Robot.STATE.IDLE;
import static org.firstinspires.ftc.teamcode.Robot.STATE.INTAKE_GLOBAL;
import static org.firstinspires.ftc.teamcode.Robot.STATE.INTAKE_RELATIVE;
import static org.firstinspires.ftc.teamcode.Robot.STATE.SCORING_GLOBAL;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.drive.basilisk.Spline;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.Storage;
import org.firstinspires.ftc.teamcode.vision.OpenCVWrapper;

@Autonomous(group = "Auto")
public class NineConeAutoRight extends LinearOpMode {
    public static final int cycles = 5;
    public static final int cycles2 = 3;
    public static int parkingNum = 0;
    public static final boolean lr = false; // Left : true | Right : false

    OpenCVWrapper openCVWrapper;

    double[] coneStackHeights = new double[]{5.05, 3.5, 3.1, 1.5, 0.8}; //5.65, 4.4, 2.75, 2.0, 0.5
    ButtonToggle toggleA = new ButtonToggle();
    double[] timeToPark = new double[]{35000, 35000, 35000};

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;
        robot.currentState = IDLE;

        openCVWrapper = new OpenCVWrapper(telemetry, hardwareMap, true);
        assert(openCVWrapper != null);

        // Signs
        int ySign = lr ? 1 : -1;

        Pose2d origin = new Pose2d(
                34,
                62 * ySign,
                lr ? Math.toRadians(-90) : Math.toRadians(90)
        );

        Pose2d toPose = new Pose2d(
                origin.getX(),
                20 * ySign,
                origin.heading
        );

        Pose2d cyclePose = new Pose2d(
                44.5, //44.5
                12 * ySign,
                Math.toRadians(180)
        );

        Pose2d cyclePose2 = new Pose2d(
                -44.5, //44.5
                12 * ySign,
                Math.toRadians(180)
        );

        robot.stayInPlacePose = cyclePose;

        Spline toPreload = new Spline(origin)
                .setReversed(true)
                .addPoint(new Pose2d(toPose.x, toPose.y, origin.heading * -1));

        Spline toCycle = new Spline(new Pose2d(toPose))
                .setReversed(true)
                .addPoint(new Pose2d(cyclePose.x, cyclePose.y, cyclePose.heading + Math.toRadians(180)));

        Spline toCycle2 = new Spline(new Pose2d(toPose))
                .addPoint(new Pose2d(cyclePose2.x, cyclePose2.y, cyclePose2.heading + Math.toRadians(180)));

        drive.setPoseEstimate(origin);

        Spline[] park = new Spline[]{
            new Spline(cyclePose2).addPoint(new Pose2d( // parking position 1
                    -59.5,
                    cyclePose2.getY(),
                    Math.toRadians(180)
            )),
            new Spline(cyclePose2).addPoint(new Pose2d( // parking position 2
                    -37,
                    cyclePose2.getY(),
                    Math.toRadians(180)
            )),
            new Spline(cyclePose2).addPoint(new Pose2d( // parking position 3
                    -12,
                    cyclePose2.getY(),
                    Math.toRadians(180)
            ))
        };

        robot.resetEncoders();

        robot.initPosition(false);

        openCVWrapper.init();
        openCVWrapper.start();

        Log.e("camera setup", "");

        sleep(2000);

        Log.e("init position", "");

        robot.claw.init();
        robot.outtake.actuation.init();

        while (opModeInInit()) {
            telemetry.setMsTransmissionInterval(50);

            boolean detected = false;

            parkingNum = openCVWrapper.getParkingNum();
            detected = true;

            if (toggleA.isToggled(gamepad1.a)) {
                robot.claw.initClose();
            }

            if (detected) {
                telemetry.addLine("Tag of interest is in sight! ID: " + (parkingNum + 1));
            } else {
                telemetry.addLine("Could not find april tag! :(");
            }

            robot.odoLifter.down();

            robot.update();
            telemetry.update();
        }

        robot.claw.close();

        robot.outtake.slides.slidesPercentMax = 1.0;

        robot.drivetrain.setBreakFollowingThresholds(new Pose2d(0.5, 0.5, Math.toRadians(-5)), cyclePose);
        drive.setPoseEstimate(origin);
        waitForStart();

        robot.claw.close();

        long startTime = System.currentTimeMillis();

        openCVWrapper.stop();

        robot.claw.close();
        robot.currentState = Robot.STATE.SCORING_GLOBAL;
        robot.startScoringGlobal(
                new Pose2d(toPose.getX(), toPose.getY()-6.0, toPose.getHeading()),
                new Pose2d(22.5, 0.0 * ySign),
                27.5);
        robot.followSpline(toPreload, this);

        while (robot.currentState == SCORING_GLOBAL || robot.currentState == DEPOSIT_AUTO) {
            robot.drivetrain.setMotorPowers(0,0,0,0);
            robot.update();
        }

        robot.currentState = INTAKE_GLOBAL;
        robot.startIntakeGlobal(
                new Pose2d(cyclePose.getX() + 0.5, cyclePose.getY(), cyclePose.getHeading()),
                new Pose2d(70.5,12 * ySign),
                coneStackHeights[0]
        );

        robot.followSpline(toCycle, this);
        robot.updateStayInPlacePID = true;

        for (int i = 0; i < cycles && (System.currentTimeMillis() - startTime <= timeToPark[parkingNum]); i++) {
            if (i != 0) {
                robot.currentState = INTAKE_GLOBAL;
                robot.startIntakeGlobal(
                        cyclePose,
                        new Pose2d(70,12 * ySign),
                        coneStackHeights[i]
                );
            }

            while ((robot.currentState == INTAKE_GLOBAL) && (System.currentTimeMillis() - startTime <= timeToPark[parkingNum])) {
                robot.update();
            }

            if (robot.sensors.robotNextToMeRight) {
                robot.startScoringGlobal(
                        cyclePose,
                        new Pose2d(21.0, 24 * ySign),
                        18.35);
            } else {
                robot.startScoringGlobal(
                        cyclePose,
                        new Pose2d(24.5, -1.0 * ySign),
                        32 + robot.autoIntakeHeightDifference);
            }

            while ((robot.currentState == SCORING_GLOBAL || robot.currentState == DEPOSIT_AUTO) && (System.currentTimeMillis() - startTime <= timeToPark[parkingNum])) {
                robot.update();
            }
        }

        robot.currentState = INTAKE_RELATIVE;

        robot.updateStayInPlacePID = false;
        robot.followSpline(toCycle2, this);
        robot.updateStayInPlacePID = true;

        for (int i = 0; i < cycles2 && (System.currentTimeMillis() - startTime <= timeToPark[parkingNum]); i++) {
            if (i != 0) {
                robot.currentState = INTAKE_GLOBAL;
                robot.startIntakeGlobal(
                        cyclePose,
                        new Pose2d(-70,12 * ySign),
                        coneStackHeights[i]
                );
            }

            while ((robot.currentState == INTAKE_GLOBAL) && (System.currentTimeMillis() - startTime <= timeToPark[parkingNum])) {
                robot.update();
            }

            if (robot.sensors.robotNextToMeRight) {
                robot.startScoringGlobal(
                        cyclePose,
                        new Pose2d(-21.0, 24 * ySign),
                        18.35);
            } else {
                robot.startScoringGlobal(
                        cyclePose,
                        new Pose2d(-24.5, -1.0 * ySign),
                        32 + robot.autoIntakeHeightDifference);
            }

            while ((robot.currentState == SCORING_GLOBAL || robot.currentState == DEPOSIT_AUTO) && (System.currentTimeMillis() - startTime <= timeToPark[parkingNum])) {
                robot.update();
            }
        }

        long timer = System.currentTimeMillis();

        robot.outtake.extension.retractExtension();
        robot.update();
        while (!(robot.outtake.slides.currentSlidesLength <= 5)) {
            robot.update();
            if (robot.outtake.extension.isInPosition(1)) {
                robot.claw.close();
            }
            if (System.currentTimeMillis() - timer >= 200) {
                robot.currentState = INTAKE_RELATIVE;
            }
        }

        robot.followSpline(park[parkingNum], this);

        Storage.autoEndPose = drive.getPoseEstimate();
        Storage.isBlue = false;
    }
}