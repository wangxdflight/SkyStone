package org.firstinspires.ftc.teamcode.drive.virtual;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.List;

public abstract class BaseDriveTrain {
    protected DcMotorEx leftFront, leftRear, rightRear, rightFront;
    protected List<DcMotorEx> drive_motors;
    protected DcMotor leftOdomEncoder, rightOdomEncoder, frontOdomEncoder;
    protected List<DcMotor> odom_motors;

    Pose2d currentPos;

    boolean usingOdom;
    boolean usingIMU;
    abstract void AddWheel(DcMotorEx motor, String name);
    abstract int GetWheelPosition(String name);
}
