package org.firstinspires.ftc.teamcode.drive.virtual;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.RobotLogger;

public class DriveTrain extends BaseDriveTrain {
    private String TAG = "DriveTrain";
    private boolean driveTrainReady = false;
    static final int DRIVE_WHEEL_NUM = 4;
    public DriveTrain() {
        super();
    }
    synchronized  public static DriveTrain getSingle_instance() {
        if (driveTrain == null)
            driveTrain = new DriveTrain();
        return driveTrain;
    }
    private static DriveTrain driveTrain = null;
    private int getMotorIndexFromName(String name) {
        int index = -1;
        if (name.equals("leftFront"))
            index = 0;
        else if (name.equals("leftRear"))
            index = 1;
        else if (name.equals("rightRear"))
            index = 2;
        else if (name.equals("rightFront"))
            index = 3;
        else
            RobotLogger.dd(TAG, "unexpected motor");
        return index;
    }
    public void AddWheel(DcMotorEx motor, String name) {
        drive_motors.set(getMotorIndexFromName(name), motor);
        if (drive_motors.size() == DRIVE_WHEEL_NUM)
        {
            driveTrainReady = true;
            RobotLogger.dd(TAG, "4 wheels are ready");
        }
    }
    public int GetWheelPosition(String name) {
        DcMotorEx motor = drive_motors.get(getMotorIndexFromName(name));
        double motor_power = motor.getPower();
        RobotLogger.dd(TAG, "drive train to figure out based on motor power");
        return 0;

    }
}
