package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.drive.roadrunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class StayInPlaceTest extends LinearOpMode {
    public static double xP, xI, xD;
    public static double yP, yI, yD;
    public static double headingP, headingI, headingD;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;

        Pose2d stayPose = new Pose2d(0, 0,0);

        xP = drive.xPID.p;
        xI = drive.xPID.i;
        xD = drive.xPID.d;
        yP = drive.yPID.p;
        yI = drive.yPID.i;
        yD = drive.yPID.d;
        headingP = drive.headingPID.p;
        headingI = drive.headingPID.i;
        headingD = drive.headingPID.d;

        robot.stayInPlacePose = stayPose;
        while (opModeInInit()) {
            robot.update();
        }
        waitForStart();
        robot.updateStayInPlacePID = true;
        while (!isStopRequested()) {
            drive.xPID.p = xP;
            drive.xPID.i = xI;
            drive.xPID.d = xD;
            drive.yPID.p = yP;
            drive.yPID.i = yI;
            drive.yPID.d = yD;
            drive.headingPID.p = headingP;
            drive.headingPID.i = headingI;
            drive.headingPID.d = headingD;
            robot.update();
        }
    }
}
