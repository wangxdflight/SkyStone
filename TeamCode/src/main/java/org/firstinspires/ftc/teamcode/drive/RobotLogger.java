package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;


public class RobotLogger {
    public static void dd(String tag, String format, Object... args)
    {
        if (DriveConstants.ENABLE_LOGGING)
            RobotLog.dd(tag, String.format(format, args));
    }
    public static void dd(String tag, String message)
    {
        if (DriveConstants.ENABLE_LOGGING)
            RobotLog.dd(tag, message);
    }

    public static void e(String format, Object... args)
    {
        if (DriveConstants.ENABLE_LOGGING)
            RobotLog.e(String.format(format, args));
    }

}
