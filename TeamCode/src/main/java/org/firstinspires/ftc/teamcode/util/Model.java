package org.firstinspires.ftc.teamcode.util;

import java.util.List;

public class Model {
    public List<Pose3D> pose3Ds;

    public Model (List<Pose3D> pose3Ds) {
        this.pose3Ds = pose3Ds;
    }

    public boolean isIntersecting (double targetX, double targetY, double targetZ) {
        double x1 = pose3Ds.get(0).getX();
        double y1 = pose3Ds.get(0).getY();
        double z1 = pose3Ds.get(0).getZ();

        double x2 = pose3Ds.get(0).getX();
        double y2 = pose3Ds.get(0).getY();
        double z2 = pose3Ds.get(0).getZ();

        if ((x1 <= targetX && targetX <= x2) && (y1 <= targetY && targetY <= y2) && (z1 <= targetZ && targetZ <= z2)) {
            return true;
        } else {
            return false;
        }
    }
}
