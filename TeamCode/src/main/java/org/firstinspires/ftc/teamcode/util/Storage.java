package org.firstinspires.ftc.teamcode.util;

/**
 * Simple static field serving as a storage medium for the bot's pose.
 * This allows different classes/opmodes to set and read from a central source of truth.
 * A static field allows data to persist between opmodes.
 */
public class Storage {
public static Pose2d autoEndPose = new Pose2d(0,0,0);
    public static boolean isBlue = true;
    public static boolean isTeleop = false;
}