package org.firstinspires.ftc.teamcode.modules.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.modules.drive.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.modules.drive.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.modules.drive.roadrunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.util.MotorPriority;
import org.firstinspires.ftc.teamcode.util.Storage;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.modules.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.modules.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.modules.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.modules.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.modules.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.modules.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.modules.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.modules.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.modules.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.modules.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.modules.drive.DriveConstants.kV;

import android.util.Log;

@Config
public class Drivetrain extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(15, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(10, 0, 0);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;

    ArrayList<MotorPriority> motorPriorities;

    public ThreeWheelLocalizer localizer;

    public Drivetrain(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        this.motorPriorities = motorPriorities;

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

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

        if (Storage.resetEncoderValues) {
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        localizer = new ThreeWheelLocalizer(hardwareMap);
        setLocalizer(localizer);

        localizer.getIMU(imu);

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }

    public void drive (Gamepad gamepad) {
        double forward = -0.4*Math.tan(((gamepad.left_stick_y * -1 ) / 0.85));
        double left = -0.4*(Math.tan(gamepad.left_stick_x / 0.85));
        double turn = gamepad.right_stick_x*0.9;

        double p1 = forward+left+turn;
        double p2 = forward-left+turn;
        double p3 = forward+left-turn;
        double p4 = forward-left-turn;
        setMotorPowers(p1, p2, p3, p4);

//        Log.e("left_stick_y:", gamepad.left_stick_y + "");
//        Log.e("left_stick_x:", gamepad.left_stick_x + "");
//        Log.e("right_stick_x:", gamepad.right_stick_x + "");
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void updateTelemetry() {}

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) {
            double forward = signal.getVel().getX() * kV + signal.getAccel().getX() * kA; forward += Math.signum(forward) * kStatic;
            double left = (signal.getVel().getY() * kV + signal.getAccel().getY() * kA) * LATERAL_MULTIPLIER * -1.0; left += Math.signum(left) * kStatic;
            double turn = signal.getVel().getHeading() * kV + signal.getAccel().getHeading() * kA; turn *= (0.5 * TRACK_WIDTH); turn += Math.signum(turn) * kStatic;

//            Log.e("FORWARD:",forward + "");
//            Log.e("LEFT:",left + "");
//            Log.e("TURN:",turn + "");

            motorPriorities.get(0).setTargetPower(forward+left-turn);
            motorPriorities.get(1).setTargetPower(forward-left-turn);
            motorPriorities.get(2).setTargetPower(forward+left+turn);
            motorPriorities.get(3).setTargetPower(forward-left+turn);
        }

        if((trajectorySequenceRunner.getLastPoseError().getX() < xThreshold) && (trajectorySequenceRunner.getLastPoseError().getY() < yThreshold) && (trajectorySequenceRunner.getLastPoseError().getHeading() < headingThreshold)) {
            breakFollowing();
            setMotorPowers(0,0,0,0);
        }

        updateTelemetry();
    }

    double xThreshold = 0.5;
    double yThreshold = 0.5;
    double headingThreshold = Math.toRadians(5.0);

    public void setBreakFollowingThresholds (Pose2d pose2d) {
        xThreshold = pose2d.getX();
        yThreshold = pose2d.getY();
        headingThreshold = pose2d.getHeading();
    }

    public void breakFollowing() {
        trajectorySequenceRunner.breakFollowing();
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
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

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double lf, double lr, double rr, double rf) {
        leftFront.setPower(lf);
        leftRear.setPower(lr);
        rightRear.setPower(rr);
        rightFront.setPower(rf);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getAngularVelocity().zRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}