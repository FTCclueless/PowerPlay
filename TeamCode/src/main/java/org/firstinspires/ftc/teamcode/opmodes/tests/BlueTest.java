package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.drive.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(group = "Test")
public class BlueTest extends LinearOpMode {
    private final int parkingNum = 3; // 1, 2, or 3
    private final int targetCycles = 5;
    private int cycles = 0;
    private int state = 0;
    private final boolean isBlue = true;
    private final boolean nearBlueTerminal = false;

    @Override
    public void runOpMode() throws InterruptedException {
        int xsign = nearBlueTerminal ? -1 : 1;
        int ysign = isBlue ? 1 : -1;

        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;

        Pose2d origin = new Pose2d(36 * xsign, 60 * ysign, 0);
        drive.setPoseEstimate(origin);

        TrajectorySequence to = drive.trajectorySequenceBuilder(origin)
            .strafeRight(48 * ysign)
            .build();

        Pose2d cyclepose = new Pose2d(36 * xsign, 12 * ysign, 0);
        TrajectorySequence cycle = drive.trajectorySequenceBuilder(cyclepose)
            .forward(24)
            .back(24)
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
        while (!isStopRequested()) {
            switch (state) {
                case 0:
                    robot.followTrajectorySequence(to);
                    state++;
                    break;
                case 1:
                    robot.followTrajectorySequence(cycle);
                    cycles++;
                    if (cycles >= targetCycles) {
                        state++;
                    }
                    break;
                case 3:
                    if (park != null) {
                        robot.followTrajectory(park);
                    }
                    break;
            }
        }
    }
}
