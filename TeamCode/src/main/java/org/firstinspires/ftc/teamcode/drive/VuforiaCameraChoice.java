package org.firstinspires.ftc.teamcode.drive.localizer;

public enum VuforiaCameraChoice
{
    PHONE_BACK(0), PHONE_FRONT(1), HUB_USB(2);
    private final int value;

    private VuforiaCameraChoice(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}