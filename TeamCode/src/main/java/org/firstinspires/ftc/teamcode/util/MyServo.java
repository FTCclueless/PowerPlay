package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Servo;

public class MyServo {
    Servo servo;
    public double speed;
    public double positionPerRadian; // 300 degrees is operating range so 1/(300*pi/180)
    double max, min, basePos;
    boolean isBackwards = false;
    public MyServo(Servo servo, String servoType, double loadMultiplier, double min, double max) {
        this(servo,servoType,loadMultiplier,min,max,min);
    }

    public MyServo(Servo servo, String servoType, double loadMultiplier, double min, double max, double basePos) {
        this(servo,servoType,loadMultiplier,min,max,min, false);
    }
    public MyServo(Servo servo, String servoType, double loadMultiplier, double min, double max, double basePos, boolean isBackwards) {
        this.servo = servo;
        this.min = min;
        this.max = max;
        this.basePos = basePos;
        this.isBackwards = isBackwards;

        switch (servoType) { // Take the no-load sp eed at 4.8 V and adjust as needed based on load on servo
            case "Torque":
                speed = Math.toRadians(60) / 0.25;
                positionPerRadian = 0.2162104887;
            break;
            case "Speed":
                speed = Math.toRadians(60) / 0.11;
                positionPerRadian = 0.2162104887;
            break;
            case "Super Speed":
                speed = Math.toRadians(60) / 0.055;
                positionPerRadian = 0.2162104887;
            break;
            case "Amazon":
                speed = Math.toRadians(60) / 0.13;
                positionPerRadian = 0.2122065908;
            break;
            case "ProModeler":
                speed = Math.toRadians(60) / 0.139;
                positionPerRadian = 0.32698;
                break;
            case "JX":
                speed = Math.toRadians(60) / 0.12;
                positionPerRadian = 0.3183098862;
                break;
        }
        speed *= loadMultiplier;

        currentPosition = basePos;
        currentAngle = basePos/positionPerRadian;

        if (isBackwards) {
            positionPerRadian *= -1;
        }
    }

    double currentAngle;
    double currentPosition;
    double offset = 0; // ex: makes deposit bucket level
    long lastUpdateTime = System.nanoTime();

    public void setPosition(double targetPosition, double power) {
        targetPosition += offset;
        targetPosition = Math.max(Math.min(targetPosition,Math.max(min,max)),Math.min(min,max));
        double targetOrientation = targetPosition / positionPerRadian; // converts position to radian angle
        double error = targetOrientation - currentAngle;
        long currentTime = System.nanoTime();
        double time = (double)(currentTime - lastUpdateTime)/1.0E9; // converts from nano to secs
        lastUpdateTime = currentTime;
        double update = Math.signum(error) * speed * power * time; // update is the distance (in radians) the servo has moved in a loop at the specified power
        currentAngle += update;
        if (Math.abs(update) >= Math.abs(error)) { // if setting servo position to update will cause the servo to go past it's target, set update = error
            currentAngle = targetPosition/positionPerRadian;
        }

        currentPosition = currentAngle * positionPerRadian; // This converts the currentAngle (in radians) to a position value (between 0-1) and then adds the intercept
        if (power == 1.0) {
            servo.setPosition(targetPosition);
        }
        else {
            servo.setPosition(currentPosition);
        }
    }

    public void setPosition (double position) {
        setPosition(position, 1.0);
    }

    public void setAngle(double angle, double power) {
        setPosition(angle * positionPerRadian * Math.signum(max-min) + basePos, power);
    }

    public void setAngle(double angle) {
        setPosition(angle * positionPerRadian * Math.signum(max-min) + basePos, 1.0);
    }
    public double getAngle() {
        return (currentAngle) + (offset-basePos)/positionPerRadian;
    }

    public double getCurrentPosition() {
        return currentPosition;
    }

//    public double clipAngle (double angle) {
//        while (angle > 2*Math.PI) {
//            angle -= 2*Math.PI * 2.0;
//        }
//        while (angle < -2*Math.PI) {
//            angle += 2*Math.PI * 2.0;
//        }
//        return angle;
//    }
}