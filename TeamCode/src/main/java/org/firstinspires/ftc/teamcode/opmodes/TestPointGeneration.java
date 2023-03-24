package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.drive.basilisk.Spline;
import org.firstinspires.ftc.teamcode.util.Pose2d;

@Autonomous(group = "Auto")
public class TestPointGeneration extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;

        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        Spline s = new Spline(new Pose2d(0, 0, 0))
                .addPoint(new Pose2d(48, 0, 0))
                .addPoint(new Pose2d(48, 20, Math.toRadians(90)));;

        waitForStart();
        while (!isStopRequested()) {
            telemetry.addData("s.points.size()", s.points.size());
            telemetry.update();
        }
    }
}
