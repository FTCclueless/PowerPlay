package org.firstinspires.ftc.teamcode.util;

import java.util.List;

public class Model {
    public List<Pose3D> pose3Ds;

    public Model (List<Pose3D> pose3Ds) {
        this.pose3Ds = pose3Ds;
    }

    public boolean isIntersecting (double targetX, double targetY, double targetZ) {
        double x1 = Math.min(pose3Ds.get(0).getX(), pose3Ds.get(1).getX());
        double y1 = Math.min(pose3Ds.get(0).getY(), pose3Ds.get(1).getY());
        double z1 = Math.min(pose3Ds.get(0).getZ(), pose3Ds.get(1).getZ());

        double x2 = Math.max(pose3Ds.get(0).getX(), pose3Ds.get(1).getX());
        double y2 = Math.max(pose3Ds.get(0).getY(), pose3Ds.get(1).getY());
        double z2 = Math.max(pose3Ds.get(0).getZ(), pose3Ds.get(1).getZ());

        if ((x1 <= targetX && targetX <= x2) && (y1 <= targetY && targetY <= y2) && (z1 <= targetZ && targetZ <= z2)) {
            return true;
        } else {
            return false;
        }
    }
}
