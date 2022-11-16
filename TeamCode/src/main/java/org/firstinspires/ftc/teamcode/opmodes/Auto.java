package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Robot.STATE.DEPOSIT;
import static org.firstinspires.ftc.teamcode.Robot.STATE.INTAKE_GLOBAL;
import static org.firstinspires.ftc.teamcode.Robot.STATE.RETRACT;
import static org.firstinspires.ftc.teamcode.Robot.STATE.SCORING_GLOBAL;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.drive.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.modules.outtake.Outtake;

@Autonomous(group = "Test")
public class Auto extends LinearOpMode {
    private static final int parkingNum = 3; // 1, 2, or 3
    private static final int targetCycles = 5;
    private static final boolean isBlue = true;
    private static final boolean nearBlueTerminal = false;
    
    // Tile offsets
    private static final int tOffsetx = -12;
    private static final int tOffsety = -8;

    /* Presets:
     * Tile len: 24
     * Robot total width: 16
     */

    @Override
    public void runOpMode() throws InterruptedException {
        int xsign = nearBlueTerminal ? -1 : 1;
        int ysign = isBlue ? 1 : -1;

        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;
        Outtake outtake = robot.outtake;

        // 48 - 8 (width / 2) = 40
        Pose2d origin = new Pose2d((48 + tOffsetx) * xsign, (72 + tOffsety) * ysign, -(Math.PI / 2));
        drive.setPoseEstimate(origin);

        TrajectorySequence to = drive.trajectorySequenceBuilder(origin)
                .strafeTo(new Vector2d((48 + tOffsetx) * xsign, (22 + tOffsety) * ysign))
                .turn(-origin.getHeading() * xsign)
                .strafeTo(new Vector2d((36 + tOffsetx) * xsign, (22 + tOffsety) * ysign)) // Half tile back
                .build();

        Trajectory cycle1 = drive.trajectoryBuilder(new Pose2d((36 + tOffsetx) * xsign, (22 + tOffsety) * ysign))
                .strafeTo(new Vector2d((72 + tOffsetx) * xsign, (22 + tOffsety) * ysign))
                .build();

        Trajectory cycle2 = drive.trajectoryBuilder(new Pose2d((72 + tOffsetx) * xsign, (22 + tOffsety) * ysign))
                .strafeTo(new Vector2d((36 + tOffsetx) * xsign, (22 + tOffsety) * ysign))
                .build();

        Vector2d parkingPos = null;
        switch (parkingNum) {
            case 1:
                parkingPos = new Vector2d((24 + tOffsetx) * xsign, (22 + tOffsety) * ysign);
                break;
            case 2:
                parkingPos = new Vector2d((48 + tOffsetx) * xsign, (22 + tOffsety) * ysign);
                break;
            case 3:
                parkingPos = new Vector2d((72 + tOffsetx) * xsign, (22 + tOffsety) * ysign);
                break;
        }

        // (72, 24) if even amount of cycles and 36 if not
        Trajectory park = null;
        Pose2d parkingOrigin = new Pose2d(36 + tOffsetx + ((targetCycles & 1) * 36), 22);
        // Miscase for when it is already in its parking space
        if (parkingPos.getX() == parkingOrigin.getX() && parkingPos.getY() == parkingOrigin.getY()) {
            park = drive.trajectoryBuilder(parkingOrigin)
                    .strafeTo(parkingPos)
                    .build();
        }

        // moves everything into init position
        robot.currentState = RETRACT;

        while (opModeInInit()) {
            robot.update();
        }

        waitForStart();

        if (!isStopRequested()) {
            robot.followTrajectorySequence(to);
            // FIXME jank
            for (int i = 0; i < targetCycles; i++) {
                robot.startIntakeGlobal(cycle1.end(),new Pose2d((72-4)*xsign,12*ysign),3-2*i);
                robot.followTrajectory(cycle1);
                while (robot.currentState == INTAKE_GLOBAL){
                    drive.update();
                }
                if (i + 1 < targetCycles) {
                    robot.startScoringGlobal(cycle2.end(),new Pose2d(24*xsign,0),30);
                    robot.followTrajectory(cycle2);
                    while (robot.currentState == SCORING_GLOBAL || robot.currentState == DEPOSIT){
                        drive.update();
                    }
                }
            }
            if (park != null) {
                robot.followTrajectory(park);
            }
        }
    }
}
