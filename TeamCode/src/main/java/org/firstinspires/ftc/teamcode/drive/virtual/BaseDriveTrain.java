package org.firstinspires.ftc.teamcode.drive.virtual;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.RobotLogger;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

public abstract class BaseDriveTrain {
    protected DcMotorEx leftFront, leftRear, rightRear, rightFront;
    protected List<DcMotorEx> drive_motors;
    protected List<DcMotorEx> odom_motors;
    protected DcMotor leftOdomEncoder, rightOdomEncoder, frontOdomEncoder;
    List<Double> lastWheelVelocities, lastWheelPositions;
    double lastExtHeading;
    MecanumDrive drive;
    Pose2d currentPos;
    private String TAG = "BaseDriveTrain";
    private static final int DRIVE_WHEEL_NUM = 4;
    boolean usingOdom;
    boolean usingIMU;
    private boolean driveTrainReady = false;
    protected Pose2d poseEstimate;

    public BaseDriveTrain(MecanumDrive _drive)
    {
        drive = _drive;
        drive_motors = new ArrayList<>();
        lastWheelVelocities = new ArrayList<>();
        lastWheelPositions = new ArrayList<>();
    }

    public int getWheelCount()
    {
        return drive_motors.size();
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
        if (getWheelCount() == DRIVE_WHEEL_NUM && name.equals("leftFront"))
        {
            RobotLogger.dd(TAG, "need to reset motors");
            drive_motors.clear();
        }
        drive_motors.add(motor);
        if (getWheelCount() == DRIVE_WHEEL_NUM)
        {
            driveTrainReady = true;
            RobotLogger.dd(TAG, "4 wheels are ready");
        }
    }
    public void setPoseEstimate(Pose2d pose)
    {
        poseEstimate = pose;
    }
    abstract public Pose2d getRobotPose() ;
}
