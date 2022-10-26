package org.firstinspires.ftc.teamcode.util;

public class ButtonToggle {
    boolean lastButton = false;

    public ButtonToggle() {}

    public boolean isClicked(boolean button) {
        boolean a = button && !lastButton;
        lastButton = button;
        return a;
    }
}