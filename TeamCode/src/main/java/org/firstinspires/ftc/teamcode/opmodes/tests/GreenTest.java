package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.drive.roadrunner.trajectorysequence.TrajectorySequence;

public class GreenTest extends LinearOpMode {
    private final int parking_num = 2; // 1, 2, or 3

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;

        Pose2d origin = new Pose2d(-48, 24, 0);
        drive.setPoseEstimate(origin);

        TrajectorySequence seq = null;
        switch (parking_num) {
            case 1:
                seq = drive.trajectorySequenceBuilder(origin)
                    .back(24)
                    .forward(48)
                    .strafeLeft(24)
                    .build();
                break;
            case 2:
                seq = drive.trajectorySequenceBuilder(origin)
                    .back(24)
                    .forward(48)
                    .build();
                break;
            case 3:
                seq = drive.trajectorySequenceBuilder(origin)
                    .back(24)
                    .forward(48)
                    .strafeRight(24)
                    .build();
                break;
        }

        waitForStart();
        if (!isStopRequested()) {
            robot.followTrajectorySequence(seq);
        }
    }
}
