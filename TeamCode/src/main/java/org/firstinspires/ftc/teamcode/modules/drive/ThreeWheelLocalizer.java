package org.firstinspires.ftc.teamcode.modules.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MyEncoder;
import org.firstinspires.ftc.teamcode.util.MyPose2d;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;

public class ThreeWheelLocalizer implements Localizer {

    public MyEncoder[] encoders;
    long lastTime = System.nanoTime();
    public double x = 0;
    public double y = 0;
    double heading = 0;

    MyPose2d currentPose = new MyPose2d(0,0,0);
    MyPose2d currentVel = new MyPose2d(0,0,0);
    MyPose2d relCurrentVel = new MyPose2d(0,0,0);
    MyPose2d currentPowerVector = new MyPose2d(0,0,0);

    ArrayList<MyPose2d> poseHistory = new ArrayList<MyPose2d>();
    ArrayList<MyPose2d> relHistory = new ArrayList<MyPose2d>();
    ArrayList<Double> loopTimes = new ArrayList<Double>();

    BNO055IMU imu;

    public ThreeWheelLocalizer(HardwareMap hardwareMap) {
        encoders = new MyEncoder[3];

        encoders[0] = new MyEncoder(new MyPose2d(1.986,7.283465),  -1); // left
        encoders[1] = new MyEncoder(new MyPose2d(1.986,-6.988189),1); // right
        encoders[2] = new MyEncoder(new MyPose2d(-3.809, -1.019),  -1); // back
    }

    public void getIMU(BNO055IMU imu){
        this.imu = imu;
    }

    public void updateEncoders(int[] encoders) {
        for (int i = 0; i < this.encoders.length; i ++){
            this.encoders[i].update(encoders[i]);
        }
    }

    public void setPose(double x, double y, double h){
        this.x = x;
        this.y = y;
        this.heading += h - this.heading;
        imuTimeStamp = System.currentTimeMillis();
        lastPose = new MyPose2d(x,y,h);
        lastImuHeading = imu.getAngularOrientation().firstAngle;
        relIMUHistory.clear();
        loopIMUTimes.clear();
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return new Pose2d(currentPose.x, currentPose.y, currentPose.heading);
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        setPose(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return new Pose2d(currentVel.x, currentVel.y, currentVel.heading);
    }

    int i = 0;
    @Override
    public void update() {
        if (i == 0){
            lastImuHeading = imu.getAngularOrientation().firstAngle;
        }
        i ++;

        long currentTime = System.nanoTime();
        double loopTime = (currentTime-lastTime)/1000000000.0;
        lastTime = currentTime;

        double deltaLeft = encoders[0].getDelta();
        double deltaRight = encoders[1].getDelta();
        double deltaBack = encoders[2].getDelta();
        double leftY = encoders[0].y;
        double rightY = encoders[1].y;
        double backX = encoders[2].x;

        //This is the heading because the heading is proportional to the difference between the left and right wheel.
        double deltaHeading = (deltaRight - deltaLeft)/(leftY-rightY);
        //This gives us deltaY because the back minus theta*R is the amount moved to the left minus the amount of movement in the back encoder due to change in heading
        double relDeltaY = deltaBack - deltaHeading*backX;
        //This is a weighted average for the amount moved forward with the weights being how far away the other one is from the center
        double relDeltaX = (deltaLeft*rightY - deltaRight*leftY)/(leftY-rightY);

        relHistory.add(0,new MyPose2d(relDeltaX,relDeltaY,deltaHeading));
        relIMUHistory.add(0,new MyPose2d(relDeltaX,relDeltaY,deltaHeading));

        if (deltaHeading != 0) { // this avoids the issue where deltaHeading = 0 and then it goes to undefined. This effectively does L'Hopital's
            double r1 = relDeltaX / deltaHeading;
            double r2 = relDeltaY / deltaHeading;
            relDeltaX = Math.sin(deltaHeading) * r1 - (1.0 - Math.cos(deltaHeading)) * r2;
            relDeltaY = (1.0 - Math.cos(deltaHeading)) * r1 + Math.sin(deltaHeading) * r2;
        }
        x += relDeltaX * Math.cos(heading) - relDeltaY * Math.sin(heading);
        y += relDeltaY * Math.cos(heading) + relDeltaX * Math.sin(heading);

        heading += deltaHeading;

        currentPose = new MyPose2d(x, y, heading);

        loopTimes.add(0,loopTime);
        loopIMUTimes.add(0,loopTime);
        poseHistory.add(0,currentPose);
        updateVelocity();
        updateHeadingIMU();
    }

    MyPose2d lastPose = new MyPose2d(0,0,0);
    long imuTimeStamp = System.currentTimeMillis();
    ArrayList<MyPose2d> relIMUHistory = new ArrayList<MyPose2d>();
    ArrayList<Double> loopIMUTimes = new ArrayList<Double>();
    double lastImuHeading = 0;

    public void updateHeadingIMU() {
        if (System.currentTimeMillis() - imuTimeStamp >= 200){
            double imuHeading = imu.getAngularOrientation().firstAngle;
            double deltaHeading = (imuHeading - lastImuHeading) - (heading - lastPose.heading);
            while (Math.abs(deltaHeading) > Math.PI){
                deltaHeading -= Math.PI * 2 * Math.signum(deltaHeading);
            }
            double sumLoopTime = 0;
            for (int i = 0; i < loopIMUTimes.size(); i ++){
                sumLoopTime += loopIMUTimes.get(i);
            }
            double backX = encoders[2].x;
            for (int i = 0; i < loopIMUTimes.size(); i ++){
                double relDeltaHeading = (loopIMUTimes.get(i)/sumLoopTime) * deltaHeading;
                relIMUHistory.get(i).y -= relDeltaHeading * backX;
                relIMUHistory.get(i).heading += relDeltaHeading;

                if (relIMUHistory.get(i).heading != 0) { // this avoids the issue where deltaHeading = 0 and then it goes to undefined. This effectively does L'Hopital's
                    double r1 = relIMUHistory.get(i).x / deltaHeading;
                    double r2 = relIMUHistory.get(i).y / deltaHeading;
                    relIMUHistory.get(i).x = Math.sin(relIMUHistory.get(i).heading) * r1 - (1.0 - Math.cos(relIMUHistory.get(i).heading)) * r2;
                    relIMUHistory.get(i).y = (1.0 - Math.cos(relIMUHistory.get(i).heading)) * r1 + Math.sin(relIMUHistory.get(i).heading) * r2;
                }
                lastPose.x += relIMUHistory.get(i).x * Math.cos(lastPose.heading) - relIMUHistory.get(i).y * Math.sin(lastPose.heading);
                lastPose.y += relIMUHistory.get(i).y * Math.cos(lastPose.heading) + relIMUHistory.get(i).x * Math.sin(lastPose.heading);

                lastPose.heading += relIMUHistory.get(i).heading;
            }
            relIMUHistory.clear();
            loopIMUTimes.clear();
            imuTimeStamp = System.currentTimeMillis();
            x = lastPose.x;
            y = lastPose.y;
            heading = lastPose.heading;
            currentPose = new MyPose2d(x,y,heading);
            lastImuHeading = imuHeading;
        }
    }

    public void updatePowerVector(double[] p){
        for (int i = 0; i < p.length; i ++){
            p[i] = Math.max(Math.min(p[i],1),-1);
        }
        double forward = (p[0] + p[1] + p[2] + p[3]) / 4;
        double left = (-p[0] + p[1] - p[2] + p[3]) / 4; //left power is less than 1 of forward power
        double turn = (-p[0] - p[1] + p[2] + p[3]) / 4;
        currentPowerVector.x = forward * Math.cos(heading) - left * Math.sin(heading);
        currentPowerVector.y = left * Math.cos(heading) + forward * Math.sin(heading);
        currentPowerVector.heading = turn;
    }

    public void updateVelocity(){
        double targetVelTimeEstimate = 0.2;
        double actualVelTime = 0;
        double relDeltaXTotal = 0;
        double relDeltaYTotal = 0;
        double totalTime = 0;
        int lastIndex = 0;
        for (int i = 0; i < loopTimes.size(); i ++){
            totalTime += loopTimes.get(i);
            if (totalTime <= targetVelTimeEstimate){
                actualVelTime += loopTimes.get(i);
                relDeltaXTotal += relHistory.get(i).getX();
                relDeltaYTotal += relHistory.get(i).getY();
                lastIndex = i;
            }
        }
        currentVel = new MyPose2d(
                (poseHistory.get(0).getX() - poseHistory.get(lastIndex).getX()) / actualVelTime,
                (poseHistory.get(0).getY() - poseHistory.get(lastIndex).getY()) / actualVelTime,
                (poseHistory.get(0).getHeading() - poseHistory.get(lastIndex).getHeading()) / actualVelTime
        );
        relCurrentVel = new MyPose2d(
                (relDeltaXTotal) / actualVelTime,
                (relDeltaYTotal) / actualVelTime,
                (poseHistory.get(0).getHeading() - poseHistory.get(lastIndex).getHeading()) / actualVelTime
        );
        while (lastIndex + 1 < loopTimes.size()){
            loopTimes.remove(loopTimes.size() - 1);
            relHistory.remove(relHistory.size() - 1);
            poseHistory.remove(poseHistory.size() - 1);
        }
    }
}
