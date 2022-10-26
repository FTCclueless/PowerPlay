package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.modules.drive.DriveConstants;


public class RobotLogger {
    public static void dd(String tag, String format, Object... args)
    {
        if (DriveConstants.ENABLE_LOGGING)
            RobotLog.dd(tag, String.format(format, args));
    }
    public static void dd(String tag, String message)
    {
        if (DriveConstants.ENABLE_LOGGING)
            RobotLog.dd(tag, message);
    }

    public static void e(String format, Object... args)
    {
        if (DriveConstants.ENABLE_LOGGING)
            RobotLog.e(String.format(format, args));
    }
    public static void callers(int level, String tag, String format, Object... args)
    {
        if (DriveConstants.ENABLE_LOGGING) {
            RobotLog.dd(tag, String.format(format, args));
            StackTraceElement[] t = Thread.currentThread().getStackTrace();
            for (int i = 0; i < level; i++) {
                int index = i+3;
                if (index < t.length) {
                    RobotLog.dd(tag, t[index].toString());
                }
            }
        }
    }

}
