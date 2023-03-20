package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Robot.STATE.DEPOSIT_AUTO;
import static org.firstinspires.ftc.teamcode.Robot.STATE.IDLE;
import static org.firstinspires.ftc.teamcode.Robot.STATE.INTAKE_GLOBAL;
import static org.firstinspires.ftc.teamcode.Robot.STATE.INTAKE_RELATIVE;
import static org.firstinspires.ftc.teamcode.Robot.STATE.SCORING_GLOBAL;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.drive.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;
import org.firstinspires.ftc.teamcode.util.Storage;
import org.firstinspires.ftc.teamcode.vision.OpenCVWrapper;

@Autonomous(group = "Auto")
public class RamAuto extends LinearOpMode {
    public static final int cycles = 5;
    public static int parkingNum = 0;
    public static final boolean lr = true; // Left : true | Right : false

    OpenCVWrapper openCVWrapper;

    double[] coneStackHeights = new double[]{4.5, 3.5, 2.6, 1.3, 0.85}; //5.65, 4.4, 2.75, 2.0, 0.5
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
                37,
                62 * ySign,
                lr ? Math.toRadians(90) : Math.toRadians(-90)
        );

        Pose2d toPose = new Pose2d(
                origin.getX(),
                -5 * ySign,
                lr ? Math.toRadians(90) : Math.toRadians(-90)
        );

        Pose2d cyclePose = new Pose2d(
                45.1,
                14 * ySign,
                Math.toRadians(180)
        );

        robot.stayInPlacePose = cyclePose;

        TrajectorySequence toRam = drive.trajectorySequenceBuilder(origin)
                .lineToConstantHeading(new Vector2d(toPose.getX(), toPose.getY()))
                .build();

        drive.setPoseEstimate(origin);

        robot.resetEncoders();

        openCVWrapper.init();
        openCVWrapper.start();

        Log.e("camera setup", "");

        sleep(2000);

        Log.e("init position", "");

        robot.claw.init();
        robot.outtake.actuation.init();

        robot.outtake.slides.slidesPercentMax = 1.0;

        drive.setPoseEstimate(origin);
        waitForStart();

        robot.claw.open();

        long startTime = System.currentTimeMillis();

        openCVWrapper.stop();

        robot.claw.close();
        robot.updateStayInPlacePID = true;
        robot.stayInPlacePose = toRam.end();
        robot.followTrajectorySequence(toRam, this);
        robot.outtake.extension.setTargetExtensionLength(30);

        long clawStart = System.currentTimeMillis();
        robot.claw.park();
        robot.outtake.actuation.init();

        while(!isStopRequested()) {
            if (System.currentTimeMillis() - clawStart >= 26000) {
                robot.outtake.extension.retractExtension();
            }
            robot.update();
        }

        robot.updateStayInPlacePID = false;

        Storage.autoEndPose = drive.getPoseEstimate();
        Storage.isBlue = true;
    }
}