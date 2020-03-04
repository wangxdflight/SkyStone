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
import org.firstinspires.ftc.teamcode.drive.RobotLogger;
import org.firstinspires.ftc.teamcode.drive.localizer.IMUBufferReader;
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
    private IMUBufferReader imuReader;
    private float lastIMU = 0;
    private boolean isStrafe = false;

    private static String TAG = "SampleMecanumDriveREV";

    public SampleMecanumDriveREV(HardwareMap hardwareMap, boolean strafe) {
        super(strafe);
        isStrafe = strafe;
        create_instance(hardwareMap);
    }


    private void create_instance(HardwareMap hardwareMap) {
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        // TODO: adjust the names of the following hardware devices to match your configuration

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        RobotLogger.dd(TAG, "SampleMecanumDriveREV created");

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DriveConstants.BRAKE_ON_ZERO ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            RobotLogger.dd(TAG, "MOTOR_VELO_PID!=0, to setPIDCoefficients %f, %f, %f" , MOTOR_VELO_PID.kP,
                    MOTOR_VELO_PID.kI, MOTOR_VELO_PID.kD);
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));

        imuReader = IMUBufferReader.getSingle_instance(hardwareMap);

        if (((DriveConstants.RUN_USING_ODOMETRY_WHEEL) && (! isStrafe)) ||
                ((DriveConstants.forceOdomInStrafe) && ( isStrafe)) ||  // new condition !!!
                ((DriveConstants.RUN_USING_ODOMETRY_WHEEL) && ( isStrafe))
        )
        {
            RobotLogger.dd(TAG, "to setLocalizer to StandardTrackingWheelLocalizer");
            setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        } else {
            setLocalizer(new MecanumLocalizer(this, DriveConstants.RUN_USING_IMU_LOCALIZER));
            RobotLogger.dd(TAG, "use default 4 wheel localizer");
        }
    }
    @Override
    public void setBrakeonZeroPower(boolean flag) {
        for (DcMotorEx motor : motors) {
            if (flag == true)
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            else
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        RobotLogger.dd(TAG, "setBrakeonZeroPower " + flag);
    }
    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        RobotLogger.dd(TAG, "getPIDCoefficients:\np: " + Double.toString(coefficients.p) + " i: " + coefficients.i
                + " d: " + coefficients.d);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        double t = getMotorVelocityF();
        RobotLogger.dd(TAG, "setPIDCoefficients:\nkP: " + Double.toString(coefficients.kP) + " kI: " + coefficients.kI
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
            //RobotLogger.dd(TAG, motor.getDeviceName() + "getWheelPositions: " + "position: " + Double.toString(t1) +
            //      " inches: " + Double.toString(t2));

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
            RobotLogger.dd(TAG, "getWheelVelocities: " + "velocity: " + Double.toString(t1) + " inches: " + Double.toString(t2));
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
        RobotLogger.dd(TAG, "setMotorPowers " + "leftFront: " + Double.toString(v));
        leftFront.setPower(v);
        RobotLogger.dd(TAG, "setMotorPowers " + "leftRear: " + Double.toString(v1));
        leftRear.setPower(v1);
        RobotLogger.dd(TAG, "setMotorPowers " + "rightRear: " + Double.toString(v2));
        rightRear.setPower(v2);
        RobotLogger.dd(TAG, "setMotorPowers" + "rightFront: " + Double.toString(v3));
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        float t = lastIMU;
        try {
            RobotLogger.dd(TAG, "to getRawExternalHeading");
            t = imuReader.getLatestIMUData();
            RobotLogger.dd(TAG, "getRawExternalHeading: " + Double.toString(t));

        } catch (Exception e) {
            e.printStackTrace();
        }
        lastIMU = t;
        return t;
    }
    public void finalize() throws Throwable {
        imuReader.cleanUP();
    }
}
