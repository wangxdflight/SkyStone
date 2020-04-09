package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class SafeSleep
{
    public static void sleep_milliseconds(LinearOpMode mode, int c) {
        if (mode.opModeIsActive()) {
            try {
                Thread.sleep(c);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}
