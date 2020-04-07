package org.firstinspires.ftc.teamcode.drive;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.STRAFE_BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.STRAFE_BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive {
    public static PIDCoefficients xTRANSLATIONAL_PID = new PIDCoefficients(DriveConstants.txP, DriveConstants.txI, DriveConstants.txD);
    public static PIDCoefficients yTRANSLATIONAL_PID = new PIDCoefficients(DriveConstants.tyP, DriveConstants.tyI, DriveConstants.tyD);
    public static PIDCoefficients HEADING_PID  = new PIDCoefficients(DriveConstants.hP, DriveConstants.hI, DriveConstants.hD);    //3, 0, 0


    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    private FtcDashboard dashboard;
    private NanoClock clock;

    private Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private DriveConstraints constraints;
    public TrajectoryFollower follower;

    private List<Pose2d> poseHistory;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private List<Double> lastWheelPositions;
    private double lastTimestamp;
    private static String TAG = "SampleMecanumDrive";
    private boolean strafe = false;
    HardwareMap hwMap;
    private BNO055IMU imu;

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        super(DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic, TRACK_WIDTH);
        hwMap = hardwareMap;
        createControllers();
    }
    public SampleMecanumDrive(HardwareMap hardwareMap, boolean s){
        super(DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic, TRACK_WIDTH);
        strafe = s;
        createControllers();
    }
    public void createControllers()
    {
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        if (strafe == false) {
            RobotLogger.dd(TAG, "using non-strafing PID, maxVel: %f, maxAccl: %f", BASE_CONSTRAINTS.maxVel, BASE_CONSTRAINTS.maxAccel);
            xTRANSLATIONAL_PID = new PIDCoefficients(DriveConstants.txP, DriveConstants.txI, DriveConstants.txD);
            yTRANSLATIONAL_PID = new PIDCoefficients(DriveConstants.tyP, DriveConstants.tyI, DriveConstants.tyD);
            HEADING_PID = new PIDCoefficients(DriveConstants.hP, DriveConstants.hI, DriveConstants.hD);

            constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        }
        else
        {
            RobotLogger.dd(TAG, "using strafing PID, maxVel: %f, maxAccl: %f", STRAFE_BASE_CONSTRAINTS.maxVel, STRAFE_BASE_CONSTRAINTS.maxAccel);
            xTRANSLATIONAL_PID = new PIDCoefficients(DriveConstants.stxP, DriveConstants.stxI, DriveConstants.stxD);
            yTRANSLATIONAL_PID = new PIDCoefficients(DriveConstants.styP, DriveConstants.styI, DriveConstants.styD);
            HEADING_PID = new PIDCoefficients(DriveConstants.shP, DriveConstants.shI, DriveConstants.shD);

            constraints = new MecanumConstraints(STRAFE_BASE_CONSTRAINTS, TRACK_WIDTH);

        }
        

        follower = new HolonomicPIDVAFollower(xTRANSLATIONAL_PID, yTRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        poseHistory = new ArrayList<>();
        RobotLogger.dd(TAG, "Mecanum drive is created");
        if (!DriveConstants.VirtualizeDrive) {
            LynxModuleUtil.ensureMinimumFirmwareVersion(hwMap);

            for (LynxModule module : hwMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }

            // TODO: adjust the names of the following hardware devices to match your configuration
            imu = hwMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);

            // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
            // upward (normal to the floor) using a command like the following:
            // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

            leftFront = hwMap.get(DcMotorEx.class, "leftFront");
            leftRear = hwMap.get(DcMotorEx.class, "leftRear");
            rightRear = hwMap.get(DcMotorEx.class, "rightRear");
            rightFront = hwMap.get(DcMotorEx.class, "rightFront");
            } else {
            leftFront = null;  // Virtual motor
            leftRear = null;
            rightRear = null;
            rightFront = null;
        }
        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        if (!DriveConstants.VirtualizeDrive) {
            for (DcMotorEx motor : motors) {
                MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
                motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
                motor.setMotorType(motorConfigurationType);
            }

            if (RUN_USING_ENCODER) {
                setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
                setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
            }
        }
        // TODO: reverse any motors using DcMotor.setDirection()

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
    }
    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getLocalizer().getPoseEstimate(), constraints);
    }
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, constraints);
    }

    public void turnAsync(double angle) {
        double heading = getPoseEstimate().getHeading();
        RobotLogger.dd(TAG, "turn: current heading "+Double.toString(heading)+" angle "+Double.toString(angle));
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
        );
        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void resetFollowerWithParameters(boolean strafe){
        RobotLogger.dd("Pre-Reinstantiate Error", follower.getLastError() + "");
        RobotLogger.dd("Follower PID Constants", strafe ? "STRAFE" : "BASE");
        if (!strafe) {
            RobotLogger.dd(TAG, "using non-strafing PID, maxVel: %f, maxAccl: %f", BASE_CONSTRAINTS.maxVel, BASE_CONSTRAINTS.maxAccel);
            xTRANSLATIONAL_PID = new PIDCoefficients(DriveConstants.txP, DriveConstants.txI, DriveConstants.txD);
            yTRANSLATIONAL_PID = new PIDCoefficients(DriveConstants.tyP, DriveConstants.tyI, DriveConstants.tyD);
            HEADING_PID = new PIDCoefficients(DriveConstants.hP, DriveConstants.hI, DriveConstants.hD);

            constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        }
        else
        {
            RobotLogger.dd(TAG, "using strafing PID, maxVel: %f, maxAccl: %f", STRAFE_BASE_CONSTRAINTS.maxVel, STRAFE_BASE_CONSTRAINTS.maxAccel);
            xTRANSLATIONAL_PID = new PIDCoefficients(DriveConstants.stxP, DriveConstants.stxI, DriveConstants.stxD);
            yTRANSLATIONAL_PID = new PIDCoefficients(DriveConstants.styP, DriveConstants.styI, DriveConstants.styD);
            HEADING_PID = new PIDCoefficients(DriveConstants.shP, DriveConstants.shI, DriveConstants.shD);

            constraints = new MecanumConstraints(STRAFE_BASE_CONSTRAINTS, TRACK_WIDTH);

        }


        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);
        follower = new HolonomicPIDVAFollower(xTRANSLATIONAL_PID, yTRANSLATIONAL_PID, HEADING_PID);
        RobotLogger.dd("STATUS", "Re-Inited Follower");
        RobotLogger.dd("Post-Reinstantiate Error", follower.getLastError() + "");
    }
    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void update() {
        RobotLogger.dd(TAG, "roadrunner control loop starts");
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        poseHistory.add(currentPose);

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError", lastError.getHeading());

        RobotLogger.dd(TAG, "update: x " + currentPose.getX());
        RobotLogger.dd(TAG, "y " + currentPose.getY());
        RobotLogger.dd(TAG, "heading " + Double.toString(currentPose.getHeading()));

        RobotLogger.dd(TAG, "xError " + lastError.getX());
        RobotLogger.dd(TAG, "yError " + lastError.getY());
        RobotLogger.dd(TAG, "headingError "  + lastError.getHeading());


        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(currentPose.getHeading());
                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));
                RobotLogger.dd(TAG, "TURN: targetOmega "+Double.toString(targetOmega)+" targetAlpha "+Double.toString(targetAlpha));
                RobotLogger.dd(TAG, "correction "+Double.toString(correction));

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose));

                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);
                DashboardUtil.drawRobot(fieldOverlay, currentPose);

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        dashboard.sendTelemetryPacket(packet);
    }
    /// new function added;
    public void setBrakeonZeroPower(boolean flag) {
    };

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, getMotorVelocityF()
            ));
        }
    }

    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        if (!DriveConstants.VirtualizeDrive) {
        for (DcMotorEx motor : motors) {
            double t1 = motor.getCurrentPosition();
            double t2 = encoderTicksToInches(t1);
            //RobotLogger.dd(TAG, motor.getDeviceName() + "getWheelPositions: " + "position: " + Double.toString(t1) +
            //      " inches: " + Double.toString(t2));

            wheelPositions.add(t2);
            }
        }
        else {
            RobotLogger.callers(6, TAG, "getWheelPositions");
            wheelPositions.add(0.0);
            wheelPositions.add(0.0);
            wheelPositions.add(0.0);
            wheelPositions.add(0.0);
        }

        return wheelPositions;
    }
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        if (!DriveConstants.VirtualizeDrive) {
            for (DcMotorEx motor : motors) {
                wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
            }
        } else {
            RobotLogger.callers(6, TAG, "getWheelVelocities");
            //Thread.dumpStack();
            wheelVelocities.add(0.0);
            wheelVelocities.add(0.0);
            wheelVelocities.add(0.0);
            wheelVelocities.add(0.0);
        }
        return wheelVelocities;
    }

    public List<Double> getMotorPowers() {
        List<Double> motorPowers = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            double t1 = motor.getPower();
            RobotLogger.dd(TAG, "getMotorPowers: " + "power: " + Double.toString(t1));
            motorPowers.add(t1);
        }
        return motorPowers;
    }
    
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        if (!DriveConstants.VirtualizeDrive) {

            leftFront.setPower(v);
            leftRear.setPower(v1);
            rightRear.setPower(v2);
            rightFront.setPower(v3);
        } else {
            RobotLogger.callers(6, TAG, "setMotorPowers");
        }
    }

    @Override
    public double getRawExternalHeading() {
        if (!DriveConstants.VirtualizeDrive) {
            return imu.getAngularOrientation().firstAngle;
        } else {
            RobotLogger.callers(8, TAG, "getRawExternalHeading");
            return 0;
        }
    }
    
        public void print_list_double(List<Double> list){
        //motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        int wheel_num = list.size();
        if (wheel_num == 4) {
            for (int i = 0; i < list.size(); i++) {
                String wheel_name = "";
                if (i == 0)
                    wheel_name = "leftFront";
                else if (i == 1)
                    wheel_name = "leftRear";
                else if (i == 2)
                    wheel_name = "rightRear";
                else if (i == 3)
                    wheel_name = "rightFront";
                else
                    wheel_name = "unexpected wheel name";

                RobotLogger.dd(TAG, wheel_name + "  " + Double.toString(list.get(i)));
            }
        } else if (wheel_num == 3)
        {
            for (int i = 0; i < list.size(); i++) {
                String wheel_name = "";
                if (i == 0)
                    wheel_name = "leftOdom";
                else if (i == 1)
                    wheel_name = "rightOdom";
                else if (i == 2)
                    wheel_name = "frontOdom";

                RobotLogger.dd(TAG, wheel_name + "  " + Double.toString(list.get(i)));
            }
        }
        else
        {
            for (int i = 0; i < list.size(); i++) {
                String wheel_name = "";
                RobotLogger.dd(TAG, wheel_name + "  " + Double.toString(list.get(i)));
            }
        }
    }
}
