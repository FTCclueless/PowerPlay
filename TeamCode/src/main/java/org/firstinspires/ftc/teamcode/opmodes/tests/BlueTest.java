package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.drive.roadrunner.trajectorysequence.TrajectorySequence;

public class BlueTest extends LinearOpMode {
    private final int parking_num = 2; // 1, 2, or 3
    private final int target_cycles = 5;
    private int cycles = 0;
    private int state = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;

        Pose2d origin = new Pose2d(48, 48, 0);
        drive.setPoseEstimate(origin);

        TrajectorySequence to = drive.trajectorySequenceBuilder(origin)
            .back(24)
            .forward(48)
            .build();

        Pose2d cyclepose = new Pose2d(-24, 48, 0);
        TrajectorySequence cycle = drive.trajectorySequenceBuilder(cyclepose)
            .strafeLeft(24)
            .strafeRight(24)
            .build();

        Trajectory park = null;
        switch (parking_num) {
            case 1:
                park = drive.trajectoryBuilder(cyclepose)
                    .strafeLeft(24)
                    .build();
                break;
            case 3:
                park = drive.trajectoryBuilder(cyclepose)
                    .strafeRight(24)
                    .build();
                break;
        }

        waitForStart();
        if (!isStopRequested()) {
            switch (state) {
                case 0:
                    robot.followTrajectorySequence(to);
                    state++;
                    break;
                case 1:
                    robot.followTrajectorySequence(cycle);
                    cycles++;
                    if (cycles >= target_cycles) {
                        state++;
                    }
                    break;
                case 3:
                    robot.followTrajectory(park);
                    break;
            }
        }
    }
}
