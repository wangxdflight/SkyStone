package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class SafeSleep
{
    public static void sleep_milliseconds(LinearOpMode mode, int c) {
        int r = c, t = 0;
        while (r > 0) {
            if (r > 200)
                t = 200;
            else
                t = r;
            if (mode.opModeIsActive()) {
                try {
                    Thread.sleep(t);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            r = r - t;
        }

    }
}
