package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.drive.basilisk.Spline;
import org.firstinspires.ftc.teamcode.util.Pose2d;

@Autonomous(group = "Auto")
public class TestBasilisk extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;

        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        Pose2d midPoint = new Pose2d(52,20,Math.toRadians(90));
        Spline s = new Spline(new Pose2d(0, 0, 0))
                .addPoint(new Pose2d(midPoint.x - 8, 0, 0))
                .addPoint(midPoint)
                .mustGoToPoint()
                .setReversed(true)
                .addPoint(new Pose2d(midPoint.x + 8,0,0))
                .addPoint(new Pose2d(midPoint.x * 2.0,0,0));

        waitForStart();
        robot.followSpline(s, this);
    }
}
