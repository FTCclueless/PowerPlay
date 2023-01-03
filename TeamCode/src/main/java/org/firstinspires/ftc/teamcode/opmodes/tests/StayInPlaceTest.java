package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.drive.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous
public class StayInPlaceTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;


        Pose2d stayPose = new Pose2d(0, 0,
                Math.toRadians(-90)
        );

        robot.stayInPlacePose = stayPose;
        while (opModeInInit()) {
            robot.update();
        }
        waitForStart();
        robot.updateStayInPlacePID = true;
        while (!isStopRequested()) {
            robot.update();
        }
    }
}
