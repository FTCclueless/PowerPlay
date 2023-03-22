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
public class AutoRight extends LinearOpMode {
    public static final int cycles = 5;
    public static int parkingNum = 0;
    public static final boolean lr = false; // Left : true | Right : false

    OpenCVWrapper openCVWrapper;

    double[] coneStackHeights = new double[]{5.25, 3.5, 3.1, 1.5, 0.8}; //5.65, 4.4, 2.75, 2.0, 0.5
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
                lr ? Math.toRadians(90) : Math.toRadians(-90)
        );

        Pose2d toPose = new Pose2d(
                origin.getX(),
                20 * ySign,
                lr ? Math.toRadians(90) : Math.toRadians(-90)
        );

        Pose2d cyclePose = new Pose2d(
                43.0, //44.5
                12 * ySign,
                Math.toRadians(180)
        );

        robot.stayInPlacePose = cyclePose;

        Spline toPreload = new Spline(origin)
                .addPoint(new Pose2d(toPose.getX(), toPose.getY()));

        Spline toCycle = new Spline(toPreload.end())
                .setReversed(true)
                .addPoint(new Pose2d(cyclePose.getX(), cyclePose.getY(), Math.toRadians(0)));

        drive.setPoseEstimate(origin);

        Spline[] park = new Spline[]{
                new Spline(cyclePose).addPoint(new Pose2d( // parking position 3
                        12,
                        cyclePose.getY()
                )).turn(Math.toRadians(95)),
                new Spline(cyclePose).addPoint(new Pose2d( // parking position 2
                        37,
                        cyclePose.getY()
                )).turn(Math.toRadians(95)),
                new Spline(cyclePose).addPoint(new Pose2d( // parking position 1
                        59.5,
                        cyclePose.getY()
                )).turn(Math.toRadians(95))
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

        robot.drivetrain.setBreakFollowingThresholds(new Pose2d(0.5, 0.5, Math.toRadians(5)), toCycle.end());
        drive.setPoseEstimate(origin);
        waitForStart();

        robot.claw.close();

        long startTime = System.currentTimeMillis();

        openCVWrapper.stop();

        robot.claw.close();
        robot.currentState = Robot.STATE.SCORING_GLOBAL;
        robot.startScoringGlobal(
                new Pose2d(toPose.getX(), toPose.getY(), toPose.getHeading()),
                new Pose2d(24.0, -3.5 * ySign),
                28.5);
        robot.followSpline(toPreload, this);

        while (robot.currentState == SCORING_GLOBAL || robot.currentState == DEPOSIT_AUTO) {
            robot.update();
        }

        robot.currentState = INTAKE_GLOBAL;
        robot.startIntakeGlobal(
                new Pose2d(cyclePose.getX() + 3, cyclePose.getY(), cyclePose.getHeading()),
                new Pose2d(70,11 * ySign),
                coneStackHeights[0]
        );

        robot.followSpline(toCycle, this);

        robot.updateStayInPlacePID = true;

        for (int i = 0; i < cycles && (System.currentTimeMillis() - startTime <= timeToPark[parkingNum]); i++) {
            if (i != 0) {
                robot.currentState = INTAKE_GLOBAL;
                robot.startIntakeGlobal(
                        toCycle.end(),
                        new Pose2d(70,11 * ySign),
                        coneStackHeights[i]
                );
            }

            while ((robot.currentState == INTAKE_GLOBAL) && (System.currentTimeMillis() - startTime <= timeToPark[parkingNum])) {
                robot.update();
            }

            if (robot.sensors.robotNextToMeRight) {
                robot.startScoringGlobal(
                        new Pose2d(toCycle.end().getX(), toCycle.end().getY(), toCycle.end().getHeading()),
                        new Pose2d(21.0, 24 * ySign),
                        18.35);
            } else {
                robot.startScoringGlobal(
                        new Pose2d(toCycle.end().getX(), toCycle.end().getY(), toCycle.end().getHeading()),
                        new Pose2d(23, -3.5 * ySign),
                        28.5);
            }

            while ((robot.currentState == SCORING_GLOBAL || robot.currentState == DEPOSIT_AUTO) && (System.currentTimeMillis() - startTime <= timeToPark[parkingNum])) {
                robot.update();
            }
        }

        robot.currentState = IDLE;
        long timer = System.currentTimeMillis();

        robot.outtake.extension.retractExtension();
        robot.update();
        while (!(robot.outtake.slides.currentSlidesLength <= 5)) {
            robot.update();
            if (robot.outtake.extension.isInPosition(1)) {
                robot.claw.close();
            }
            if (System.currentTimeMillis() - timer >= 750) {
                robot.currentState = INTAKE_RELATIVE;
            }
        }

        robot.updateStayInPlacePID = false;
        robot.drivetrain.setBreakFollowingThresholds(new Pose2d(0.5, 0.5, Math.toRadians(5)), park[parkingNum].end());

        robot.followSpline(park[parkingNum], this);

        long clawStart = System.currentTimeMillis();
        robot.claw.park();
        robot.outtake.actuation.init();

        do  {
            robot.claw.park();
            robot.outtake.actuation.init();
            robot.update();
        } while (System.currentTimeMillis() - clawStart <= 2000);

        Storage.autoEndPose = drive.getPoseEstimate();
        Storage.isBlue = false;
    }
}