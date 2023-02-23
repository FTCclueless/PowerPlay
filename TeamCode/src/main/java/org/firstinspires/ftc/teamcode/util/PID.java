package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

public class PID {
    public double p;
    public double i;
    public double d;
    public PID(double P, double I, double D){
        p=P;
        i=I;
        d=D;
    }
    double integral = 0;
    long lastLoopTime = System.nanoTime();
    double lastError = 0;
    int counter = 0;
    public double loopTime = 0.0;

    public void resetIntegral() {
        integral = 0;
    }

    public double update(double error){
        if (counter == 0) {
            lastLoopTime = System.nanoTime() - 10000000;
        }

        long currentTime = System.nanoTime();
        loopTime = (currentTime - lastLoopTime)/1000000000.0;
        lastLoopTime = currentTime; // lastLoopTime's start time

        double proportion = p * error;
        integral += error * i * loopTime;
        double derivative = d * (error - lastError)/loopTime;

        lastError = error;
        counter ++;

        return proportion + integral + derivative;
    }

    public void updatePID(PID newPid) {
        this.p = newPid.p;
        this.i = newPid.i;
        this.d = newPid.d;
    }
}