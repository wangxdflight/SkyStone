package org.firstinspires.ftc.teamcode.drive.virtual;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_RPM;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.WHEEL_RADIUS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMaxRpm;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

// motor to wheel
public class DriveTrain extends BaseDriveTrain {
    private String TAG = "DriveTrain";
    private boolean driveTrainReady = false;
    private int wheel_count = 0;
    static final int DRIVE_WHEEL_NUM = 4;
    private static DriveTrain driveTrain_singleInstance = null;

    public DriveTrain() {
        super();
        drive_motors = Arrays.asList(new VirtualMotorEx[DRIVE_WHEEL_NUM]);
    }
    synchronized  public static DriveTrain getSingle_instance() {
        if (driveTrain_singleInstance == null) {
            RobotLogger.dd("DriveTrain", "drive train created");
            driveTrain_singleInstance = new DriveTrain();
        }
        else
            RobotLogger.dd("DriveTrain", "drive train already exists");
        return driveTrain_singleInstance;
    }
    private int getMotorIndexFromName(String name) {
        int index = -1;
        if (name.equalsIgnoreCase("leftFront"))
            index = 0;
        else if (name.equalsIgnoreCase("leftRear"))
            index = 1;
        else if (name.equalsIgnoreCase("rightRear"))
            index = 2;
        else if (name.equalsIgnoreCase("rightFront"))
            index = 3;
        else
            RobotLogger.dd(TAG, "unexpected motor");
        return index;
    }
    public void AddWheel(DcMotorEx motor, String name) {
        RobotLogger.dd(TAG, "add wheel: " + name);
        drive_motors.set(getMotorIndexFromName(name), motor);
        wheel_count ++;
        if (wheel_count == DRIVE_WHEEL_NUM)
        {
            driveTrainReady = true;
            RobotLogger.dd(TAG, "4 wheels are ready");
        }
    }


    public void finalize() throws Throwable{
        RobotLogger.dd(TAG, "drive train is finalize");
        driveTrain_singleInstance = null;
    }
}
