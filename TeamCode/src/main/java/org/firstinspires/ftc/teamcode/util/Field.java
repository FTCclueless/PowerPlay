package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Field {
    Pole[][] poles = new Pole[][]{
            {new Pole(48, 48, 0), new Pole(48, 24, 1), new Pole(48, 0, 0), new Pole(48, -24, 1), new Pole(48, -48, 0)},
            {new Pole(24, 48, 1), new Pole(24, 24, 2), new Pole(24, 0, 3), new Pole(24, -24, 2), new Pole(24, -48, 1)},
            {new Pole(0, 48, 0), new Pole(0, 24, 3), new Pole(0, 0, 0), new Pole(0, -24, 3), new Pole(0, -48, 0)},
            {new Pole(-24, 48, 1), new Pole(-24, 24, 2), new Pole(-24, 0, 3), new Pole(-24, -24, 2), new Pole(-24, -48, 1)},
            {new Pole(-48, 48, 0), new Pole(-48, 24, 1), new Pole(-48, 0, 0), new Pole(-48, -24, 1), new Pole(-48, -48, 0)}
    };

    public Pose2d getNearestPole (Pose2d armPose) {
        double armX = armPose.getX();
        double armY = armPose.getY();

        Pole closetPole = new Pole(0,0,0);
        double closetDistance = 16384;

        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                double currentDistance = Math.pow(armX - poles[i][j].x, 2) + Math.pow(armY - poles[i][j].y, 2);
                if (currentDistance < closetDistance) {
                    closetDistance = currentDistance;
                    closetPole = poles[i][j];
                }
            }
        }

        return new Pose2d(closetPole.x, closetPole.y);
    }

    public Pose2d getNearestPole (Pose2d armPose, int heightLevel) {
        double armX = armPose.getX();
        double armY = armPose.getY();

        Pole closetPole = new Pole(0,0,0);
        double closetDistance = 16384;

        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                double currentDistance = Math.pow(armX - poles[i][j].x, 2) + Math.pow(armY - poles[i][j].y, 2);
                if ((currentDistance < closetDistance) && (poles[i][j].heightLevel == heightLevel)) {
                    closetDistance = currentDistance;
                    closetPole = poles[i][j];
                }
            }
        }

        return new Pose2d(closetPole.x, closetPole.y);
    }
}
