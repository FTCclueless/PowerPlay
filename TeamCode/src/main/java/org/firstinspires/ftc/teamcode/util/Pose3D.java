package org.firstinspires.ftc.teamcode.util;

public class Pose3D {
    public double x;
    public double y;
    public double z;

    public Pose3D (double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }
    public Pose3D (Pose3D pose3d){
        this.x = pose3d.x;
        this.y = pose3d.y;
        this.z = pose3d.z;

    }
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getZ(){
        return z;
    }
}
