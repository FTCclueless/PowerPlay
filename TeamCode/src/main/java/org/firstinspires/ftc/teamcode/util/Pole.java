package org.firstinspires.ftc.teamcode.util;

public class Pole {
    public double x;
    public double y;
    public int heightLevel; // 0 = ground, 1 = low, 2 = medium, 3 = high

    public Pole (double x, double y, int h) {
        this.x = x;
        this.y = y;
        this.heightLevel = h;
    }
}
