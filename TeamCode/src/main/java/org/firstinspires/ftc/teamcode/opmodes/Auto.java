package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.drive.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(group = "Test")
public class Auto extends LinearOpMode {
    private final int parkingNum = 3; // 1, 2, or 3
    private final int targetCycles = 5;
    private final boolean isBlue = true;
    private final boolean nearBlueTerminal = false;

    @Override
    public void runOpMode() throws InterruptedException {
        int xsign = nearBlueTerminal ? -1 : 1;
        int ysign = isBlue ? 1 : -1;

        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;

        Pose2d origin = new Pose2d((48 - 8) * xsign, 72 * ysign, 0);
        drive.setPoseEstimate(origin);

        TrajectorySequence to = drive.trajectorySequenceBuilder(origin)
            .forward(49 * ysign)
            .build();

        Pose2d cyclepose = new Pose2d((48 - 8) * xsign, 23 * ysign, 0);
        TrajectorySequence cycle = drive.trajectorySequenceBuilder(cyclepose)
            .forward(24)
            .back(24) // Go back a half tile more
            .build();

        Trajectory park = null;
        switch (parkingNum) {
            case 1:
                park = drive.trajectoryBuilder(cyclepose)
                    .forward(24)
                    .build();
                break;
            case 3:
                park = drive.trajectoryBuilder(cyclepose)
                    .back(24)
                    .build();
                break;
        }

        waitForStart();

        if (!isStopRequested()) {
            robot.followTrajectorySequence(to);
            for (int i = 0; i < targetCycles; i++) {
                robot.followTrajectorySequence(cycle);
            }
            if (park != null) {
                robot.followTrajectory(park);
            }
        }
    }
}
