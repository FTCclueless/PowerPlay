package org.firstinspires.ftc.teamcode.modules.drive;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.modules.drive.basilisk.Spline;
import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.util.MotorPriority;
import org.firstinspires.ftc.teamcode.util.PID;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.modules.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.modules.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.modules.drive.DriveConstants.kStatic;

import android.util.Log;

@Config
public class Drivetrain {
//    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(12.5, 0, 0);
//    public static PIDCoefficients HEADING_PID = new PIDCoefficients(10, 0, 0);

//    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(15, 0, 0);
//    public static PIDCoefficients HEADING_PID = new PIDCoefficients(10, 0, 0);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;

    ArrayList<MotorPriority> motorPriorities;

    public ThreeWheelLocalizer localizer;

    public Spline currentSplineToFollow = new Spline(new Pose2d(0,0,0));

    public PID autoPID = new PID(0.0, 0.0,0.0);

    public Drivetrain(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities) {
        this.motorPriorities = motorPriorities;

        currentSplineToFollow.points.clear();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        imu = hardwareMap.get(BNO055IMU.class, "imu1");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Z);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (int i = 0; i < motors.size(); i ++) {
            MotorConfigurationType motorConfigurationType = motors.get(i).getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motors.get(i).setMotorType(motorConfigurationType);
            motorPriorities.add(new MotorPriority(motors.get(i),3,5));
        }

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        localizer = new ThreeWheelLocalizer(hardwareMap);

        localizer.getIMU(imu);
    }

    public void resetOdoReadings() {
        localizer.updateEncoders(new int[]{0,0,0});
    }

    public void drive (Gamepad gamepad) {
        double forward = -0.4*Math.tan(((gamepad.left_stick_y * -1 ) / 0.85));
        double left = -0.4*(Math.tan(gamepad.left_stick_x / 0.85)) * 0.8;
        double turn = gamepad.right_stick_x*0.9;

        double p1 = forward+left+turn;
        double p2 = forward-left+turn;
        double p3 = forward+left-turn;
        double p4 = forward-left-turn;
        setMotorPowers(p1, p2, p3, p4);
    }

    public void updateTelemetry() {}

    double xThreshold = 0.5;
    double yThreshold = 0.5;
    double headingThreshold = Math.toRadians(5.0);

    public void update() {
        localizer.update();
        Pose2d estimate = localizer.getPoseEstimate();
        Pose2d signal = currentSplineToFollow.getErrorFromNextPoint(estimate); // signal is null when in teleop only in auto do we have signal
        if (signal != null) {
            double errorDistance = Math.sqrt(Math.pow(signal.x,2) + Math.pow(signal.y,2));
            boolean inDist = errorDistance < 6.0;
            double headingError = inDist ? signal.heading : Math.atan2(signal.y*4.0,signal.x)+currentSplineToFollow.points.get(0).headingOffset; // pointing at the point
            while (Math.abs(headingError) > Math.toRadians(180)) {
                headingError -= Math.signum(headingError) * Math.toRadians(360);
            }
            Log.e("headingError", headingError + " " + inDist);
            double fwd = signal.x;
            double strafePower = inDist ? signal.y/-4.0 : 0.0;
            double turn = 2.5*headingError*TRACK_WIDTH/2.0; // s/r = theta
            double[] motorPowers = {
                    fwd + strafePower - turn,
                    fwd - strafePower - turn,
                    fwd + strafePower + turn,
                    fwd - strafePower + turn
            };
            double max = Math.abs(motorPowers[0]);
            for (int i = 1; i < motorPowers.length; i ++) {
                max = Math.max(max, Math.abs(motorPowers[i]));
            }
            double maxSpeed = Math.min(1.0, errorDistance / currentSplineToFollow.minimumRobotDistanceFromPoint); // we want the speed to slow down as we approach the point
           // slow down on turns
            if (currentSplineToFollow.points.size() >= 2) {
                double changeAngle = 0;
                int maxDist = 12; // start slowing down 12 inches from turn
                double k = 0.35;
                int maxIndex = Math.min(maxDist/2-1,currentSplineToFollow.points.size()-1); // since we have points every two inches if we divide maxDist by 2 we get number of points
                for (int i = 0; i < maxIndex; i ++) { // iterating through the next 6 points and taking a kalman filter of change angles
                    // first get change in angle between last point and point in front of it then multiply by a constant k and add in 65 percent of change angle from previous point
                    changeAngle = Math.abs(currentSplineToFollow.points.get(maxIndex - i).heading - currentSplineToFollow.points.get(maxIndex-i-1).heading) * k + changeAngle * (1.0-k);
                }
                maxSpeed *= 1.0 - Math.min(changeAngle/Math.toRadians(5),0.15); // max we slow down to on a turn is 0.85
            }
            maxSpeed = Math.max(maxSpeed,0.5); // min max speed
            for (int i = 0; i < motorPowers.length; i ++){
                motorPowers[i] /= max; // keeps proportions in tack by getting a percentage
                motorPowers[i] *= maxSpeed; // slow down motors
                motorPowers[i] *= 1.0-kStatic; // we do this so that we keep proportions when we add kstatic in the next line below. If we had just added kstatic without doing this 0.9 and 1.0 become the same motor power
                motorPowers[i] += kStatic * Math.signum(motorPowers[i]);
                motorPriorities.get(i).setTargetPower(motorPowers[i]);
            }
        }

        if((breakFollowing)
                && (Math.abs(estimate.getX() - targetPose.getX()) < xThreshold)
                && (Math.abs(estimate.getY() - targetPose.getY()) < yThreshold)
                && (Math.abs(estimate.getHeading() - targetPose.getHeading()) < headingThreshold)) {
            breakFollowing();
            setMotorPowers(0,0,0,0);
        }

        updateTelemetry();
    }

    public PID xPID = new PID(0.08, 0.008,0.0);
    public PID yPID = new PID(-0.15, -0.015,0.0);
    public PID headingPID = new PID(1.25, 0.1,0.0);

    public void updatePID(Pose2d targetPose) {
        Pose2d robotPose = localizer.getPoseEstimate();
        double deltaX = targetPose.getX() - robotPose.getX();
        double deltaY = targetPose.getY() - robotPose.getY();

        double xError = deltaX * Math.cos(robotPose.getHeading()) + deltaY * Math.sin(robotPose.getHeading());
        double yError = deltaY * Math.cos(robotPose.getHeading()) - deltaX * Math.sin(robotPose.getHeading());
        double headingError = targetPose.getHeading() - robotPose.getHeading();
        while(Math.abs(headingError) > Math.PI ){
            headingError -= Math.PI * 2 * Math.signum(headingError);
        }
//        double headingError = robotPose.getHeading() - targetPose.getHeading();
        TelemetryUtil.packet.put("targetHeading: ", targetPose.getHeading());
        TelemetryUtil.packet.put("robotHeading: ", robotPose.getHeading());
        TelemetryUtil.packet.put("headingError: ", headingError);

        double forward = xPID.update(xError);
        double left = yPID.update(yError);
        double turn = headingPID.update(headingError);

        motorPriorities.get(0).setTargetPower(forward+left-turn);
        motorPriorities.get(1).setTargetPower(forward-left-turn);
        motorPriorities.get(2).setTargetPower(forward+left+turn);
        motorPriorities.get(3).setTargetPower(forward-left+turn);
    }

    boolean breakFollowing = false;
    Pose2d targetPose = new Pose2d(0,0,0);

    public void setBreakFollowingThresholds (Pose2d thresholds, Pose2d targetPose) {
        this.targetPose = targetPose;
        breakFollowing = true;
        xThreshold = thresholds.getX();
        yThreshold = thresholds.getY();
        headingThreshold = thresholds.getHeading();
    }

    public void breakFollowing() {
        currentSplineToFollow.points.clear();
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return currentSplineToFollow.points.size() != 0;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setMotorPowers(double lf, double lr, double rr, double rf) {
        leftFront.setPower(lf);
        leftRear.setPower(lr);
        rightRear.setPower(rr);
        rightFront.setPower(rf);
    }

    public Pose2d getPoseEstimate() {
        return localizer.getPoseEstimate();
    }

    public void setSpline(Spline spline) {
        currentSplineToFollow = spline;
    }

    public void setPoseEstimate(Pose2d pose2d) {
        localizer.setPoseEstimate(pose2d);
    }
}