package org.firstinspires.ftc.teamcode.drive.mecanum;

import android.support.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;
import org.firstinspires.ftc.teamcode.drive.localizer.StandardTrackingWheelLocalizer;
import com.qualcomm.robotcore.util.RobotLog;

/*
 * Simple mecanum drive hardware implementation for REV hardware. If your hardware configuration
 * satisfies the requirements, SampleMecanumDriveREVOptimized is highly recommended.
 */
public class SampleMecanumDriveREV extends SampleMecanumDriveBase {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;
    private String TAG = "SampleMecanumDriveREV";
    public SampleMecanumDriveREV(HardwareMap hardwareMap, boolean strafe) {
        super(strafe);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        // TODO: adjust the names of the following hardware devices to match your configuration

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        RobotLog.dd(TAG, "SampleMecanumDriveREV created");

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DriveConstants.BRAKE_ON_ZERO?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            RobotLog.dd(TAG,"MOTOR_VELO_PID!=0, to setPIDCoefficients " + Double.toString(MOTOR_VELO_PID.kP)
            + " " + Double.toString(MOTOR_VELO_PID.kI) + " " + Double.toString(MOTOR_VELO_PID.kD));
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE); //???
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        if (DriveConstants.RUN_USING_ODOMETRY_WHEEL) {
            RobotLog.dd(TAG, "to setLocalizer to StandardTrackingWheelLocalizer");
            setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap, imu));
        }
        else
            RobotLog.dd(TAG, "not using Odometry localizer");
    }
    @Override
    public void setBrakeonZeroPower(boolean flag) {
        for (DcMotorEx motor : motors) {
            if (flag == true)
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            else
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        RobotLog.dd(TAG, "setBrakeonZeroPower " + flag);
    }
    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        RobotLog.dd(TAG, "getPIDCoefficients:\np: " + Double.toString(coefficients.p) + " i: " + coefficients.i
                + " d: " + coefficients.d);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        double t = getMotorVelocityF();
        RobotLog.dd(TAG, "setPIDCoefficients:\nkP: " + Double.toString(coefficients.kP) + " kI: " + coefficients.kI
                + " kD: " + coefficients.kD + " MotorVelocityF: " + Double.toString(t));
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, t
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            double t1 = motor.getCurrentPosition();
            double t2 = encoderTicksToInches(t1);
            RobotLog.dd(TAG, "getWheelPositions: " + "position: " + Double.toString(t1) + " inches: " + Double.toString(t2));

            wheelPositions.add(t2);
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            double t1 = motor.getVelocity();
            double t2 = encoderTicksToInches(t1);
            RobotLog.dd(TAG, "getWheelVelocities: " + "velocity: " + Double.toString(t1) + " inches: " + Double.toString(t2));
            wheelVelocities.add(t2);
        }
        return wheelVelocities;
    }

    @Override
    public List<Double> getMotorPowers(List<DcMotorEx> motors) {
        List<Double> motorPowers = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            double t = motor.getPower();
            RobotLog.dd(TAG, "getMotorPowers: " + "power: " + Double.toString(t));

            motorPowers.add(t);
        }
        return motorPowers;
    }
    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        RobotLog.dd(TAG, "setMotorPowers "+"leftFront: " + Double.toString(v));
        RobotLog.dd(TAG, "setMotorPowers "+"leftRear: "+Double.toString(v1));
        RobotLog.dd(TAG, "setMotorPowers "+"rightRear: "+Double.toString(v2));
        RobotLog.dd(TAG, "setMotorPowers"+"rightFront: "+Double.toString(v3));
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        double t = imu.getAngularOrientation().firstAngle;
        RobotLog.dd(TAG, "getRawExternalHeading: " + Double.toString(t));
        return t;
    }
}
