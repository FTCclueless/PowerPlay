package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.drive.roadrunner.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(group = "Test")
public class PurpleTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;

        Pose2d origin = new Pose2d(36, -60, 0);
        drive.setPoseEstimate(origin);

        TrajectorySequence seq = drive.trajectorySequenceBuilder(origin)
                .forward(24)
                .build();

        waitForStart();
        if (!isStopRequested()) {
            robot.followTrajectorySequence(seq);
        }
    }
}
