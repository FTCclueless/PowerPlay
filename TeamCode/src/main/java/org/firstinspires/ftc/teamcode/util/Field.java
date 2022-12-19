package org.firstinspires.ftc.teamcode.util;

public class Field {
    Pole[][] poles;

    public Field () {
        poles = new Pole[][]{
                {new Pole(48, 48, 0), new Pole(48, 24, 1), new Pole(48, 0, 0), new Pole(48, -24, 1), new Pole(48, -48, 0)},
                {new Pole(24, 48, 1), new Pole(24, 24, 2), new Pole(24, 0, 3), new Pole(24, -24, 2), new Pole(24, -48, 1)},
                {new Pole(0, 48, 0), new Pole(0, 24, 3), new Pole(0, 0, 0), new Pole(0, -24, 3), new Pole(0, -48, 0)},
                {new Pole(-24, 48, 1), new Pole(-24, 24, 2), new Pole(-24, 0, 3), new Pole(-24, -24, 2), new Pole(-24, -48, 1)},
                {new Pole(-48, 48, 0), new Pole(-48, 24, 1), new Pole(-48, 0, 0), new Pole(-48, -24, 1), new Pole(-48, -48, 0)}
        };
    }

    public Pole getNearestPole (double armX, double armY) {
        Pole closetPole = null;
        double closetDistance = 128;

        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                double currentDistance = Math.pow(armX - poles[i][j].x, 2) + Math.pow(armY - poles[i][j].y, 2);
                if (currentDistance < closetDistance) {
                    closetDistance = currentDistance;
                    closetPole = poles[i][j];
                }
            }
        }

        return closetPole;
    }
}
