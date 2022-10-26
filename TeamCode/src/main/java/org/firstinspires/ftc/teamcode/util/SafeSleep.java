package org.firstinspires.ftc.teamcode.util;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class SafeSleep
{
    public static void sleep_milliseconds(LinearOpMode mode, int c) {
        long start_time = SystemClock.elapsedRealtime();
        int r = c, t = 0;
        while (r > 0) {
            if (r > 100)
                t = 100;
            else
                t = r;
            if (!mode.isStopRequested()) {
                try {
                    Thread.sleep(t);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            r = r - t;
        }
        long sleep_duration = SystemClock.elapsedRealtime() - start_time;
        //RobotLogger.dd("SafeSleep", "actually slept for " + sleep_duration + " when requested to sleep " + c);
    }
}
