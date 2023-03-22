package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.drive.basilisk.Spline;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.Storage;
import org.firstinspires.ftc.teamcode.vision.OpenCVWrapper;

// runOpMode isn't static. The constructor must be called.
public class ParkAuto extends LinearOpMode {
    private int parkingNum = 2;
    protected boolean lr = true; // Left : true | Right : false
    protected boolean tb = false; // Top : true | Bottom : false

    private OpenCVWrapper openCVWrapper;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;
        openCVWrapper = new OpenCVWrapper(telemetry, hardwareMap, true);
        assert(openCVWrapper != null);

        // Signs
        int xSign = tb ? 1 : -1;
        int ySign = lr ? 1 : -1;

        Pose2d origin = new Pose2d(
                36 * xSign,
                60 * ySign,
                Math.PI / 2 * (!lr ? -1 : 1)
        );

        drive.setPoseEstimate(origin);

        drive.setPoseEstimate(origin);
        Spline to = new Spline(origin)
            .addPoint(new Pose2d(
                origin.getX(),
                20 * ySign
            ));

        Spline[] park = new Spline[]{
                new Spline(to.end()).addPoint(new Pose2d( // parking position 1
                        59.5 * xSign,
                        20 * ySign
                )),
                new Spline(to.end()).addPoint(new Pose2d( // parking position 2
                        34 * xSign,
                        20 * ySign
                )),
                new Spline(to.end()).addPoint(new Pose2d( // parking position 3
                        13 * xSign,
                        20 * ySign
                ))
        };

        openCVWrapper.init();
        openCVWrapper.start();

        while (opModeInInit()) {
            telemetry.setMsTransmissionInterval(50);

            boolean detected = false;

            /////
            parkingNum = openCVWrapper.getParkingNum();
            detected = true;  //should we always set to true ??? It is only used to send telemetry anyways
            ///////////

            robot.update();

            if (detected) {
                telemetry.addLine(String.format("Tag of interest is in sight! ID: %d", parkingNum + 1));
            } else {
                telemetry.addLine("Could not find april tag! :(");
            }

            telemetry.update();
        }

        waitForStart();

        openCVWrapper.stop();

        robot.followSpline(to, this);
        robot.followSplineWithTimer(park[parkingNum], this);

        Storage.autoEndPose = drive.getPoseEstimate();
        Storage.isBlue = true;
    }
}