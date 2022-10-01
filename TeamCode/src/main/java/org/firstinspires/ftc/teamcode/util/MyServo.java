package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Servo;

public class MyServo {
    Servo servo;
    public double speed;
    public MyServo(Servo servo, String servoType, double loadMultiplier, double ticksPerRadians, double intercept, double min, double max) {
        this.servo = servo;
        this.positionPerRadian = ticksPerRadians;
        this.intercept = intercept;
        this.min = min;
        this.max = max;

        switch (servoType) { // Take the no-load speed at 4.8 V and adjust as needed based on load on servo
            case "Torque": speed = Math.toRadians(60) / 0.25; break;
            case "Speed": speed = Math.toRadians(60) / 0.11; break;
            case "Super Speed": speed = Math.toRadians(60) / 0.055; break;
        }
        speed *= loadMultiplier;
    }
    double currentAngle = 0;
    double currentPosition = 0;
    double offset = 0; // ex: makes deposit bucket level
    long lastUpdateTime = System.nanoTime();
    public double positionPerRadian = 0.19098593171; //300 degrees is operating range so 1/(300*pi/180)
    public double intercept = 0; // reassigns what 0 is for the servo
    double max = 1, min = 0;

    double lastPos = 0.0;
    double currentPos = 0.0;
    long timeSinceNewPos = System.currentTimeMillis();
    boolean isMoving = false;
    double approxTime = 0.0;

    public void setPosition(double targetPosition, double power) {
        targetPosition += offset;
        targetPosition = Math.max(Math.min(targetPosition,max),min);
        double targetOrientation = (targetPosition - intercept) / positionPerRadian;
        double error = targetOrientation - currentAngle;
        long currentTime = System.nanoTime();
        double time = (double)(currentTime - lastUpdateTime)/1.0E9;
        lastUpdateTime = currentTime;
        double update = Math.signum(error) * speed * power * time; // update is the distance (in radians) the servo will move in a loop at the specified power
        if (Math.abs(update) >= Math.abs(error)) { // if setting servo position to update will cause the servo to go past it's target, set update = error
            update = error;
        }
        currentAngle += update;
        currentPosition = currentAngle * positionPerRadian + intercept; // This converts the currentAngle (in radians) to a position value (between 0-1) and then adds the intercept
        if (power == 1.0) {
            servo.setPosition(targetPosition);
        }
        else{
            servo.setPosition(currentPosition);
        }
    }
    public void setAngle(double orientation, double power){
        setPosition(orientation * positionPerRadian + intercept,power);
    }
    public void setPositionOrientation(double orientation){
        setPosition(orientation * positionPerRadian + intercept,1.0);
    }
    public void setAngle(double orientation){
        setPosition(orientation * positionPerRadian + intercept,1.0);
    }
    public double getAngle() {
        return currentAngle + offset/positionPerRadian;
    }

    public void setPositionTimeBased(double position) {
        if(position != currentPos && !isMoving) {
            timeSinceNewPos = System.currentTimeMillis();

            double setPosition = ((max-min) * position + min); // desmos

            servo.setPosition(setPosition);
            isMoving = true;

            approxTime = 1500*(Math.abs(position - lastPos));
        }

        if(((System.currentTimeMillis() - timeSinceNewPos) >= approxTime) && position != currentPos) {
            currentPos = position;
            lastPos = position;
            isMoving = false;
        }
    }

    public double getCurrentPosition() {
        return currentPos;
    }
    public double getLastPos() { return lastPos; }

}