package org.firstinspires.ftc.teamcode.drive.mecanum;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.Drive;
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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.ArrayList;
import java.util.List;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.BASE_CONSTRAINTS;

/*
 * Base class with shared functionality for sample mecanum drives. All hardware-specific details are
 * handled in subclasses.
 */
@Config
public abstract class SampleMecanumDriveBase extends MecanumDrive {
    public static PIDCoefficients xTRANSLATIONAL_PID = new PIDCoefficients(DriveConstants.txP, DriveConstants.txI, DriveConstants.txD);
    public static PIDCoefficients yTRANSLATIONAL_PID = new PIDCoefficients(DriveConstants.tyP, DriveConstants.tyI, DriveConstants.tyD);
    public static PIDCoefficients HEADING_PID  = new PIDCoefficients(DriveConstants.hP, DriveConstants.hI, DriveConstants.hD);    //3, 0, 0

    private String TAG = "SampleMecanumDriveBase";

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
    private TrajectoryFollower follower;

    private List<Double> lastWheelPositions;
    private double lastTimestamp;

    public SampleMecanumDriveBase() {
        super(DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic, DriveConstants.TRACK_WIDTH);
        RobotLog.dd(TAG, "kV "+Double.toString(DriveConstants.kV)+" kA "+Double.toString(DriveConstants.kA)+" kStatic "+Double.toString(DriveConstants.kStatic));
        RobotLog.dd(TAG, "TRACK_WIDTH "+Double.toString(DriveConstants.TRACK_WIDTH));
        RobotLog.dd(TAG, "txP "+Double.toString(DriveConstants.txP)+" txI "+Double.toString(DriveConstants.txI)+" txD "+Double.toString(DriveConstants.txD));
        RobotLog.dd(TAG, "tyP "+Double.toString(DriveConstants.tyP)+" tyI "+Double.toString(DriveConstants.tyI)+" tyD "+Double.toString(DriveConstants.tyD));
        RobotLog.dd(TAG, "hP "+Double.toString(DriveConstants.hP)+" hI "+Double.toString(DriveConstants.hI)+" hD "+Double.toString(DriveConstants.hD));

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = Mode.IDLE;

        xTRANSLATIONAL_PID = new PIDCoefficients(DriveConstants.txP, DriveConstants.txI, DriveConstants.txD);
        yTRANSLATIONAL_PID = new PIDCoefficients(DriveConstants.tyP, DriveConstants.tyI, DriveConstants.tyD);
        HEADING_PID = new PIDCoefficients(DriveConstants.hP, DriveConstants.hI, DriveConstants.hD);

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraints = new MecanumConstraints(BASE_CONSTRAINTS, DriveConstants.TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(xTRANSLATIONAL_PID, yTRANSLATIONAL_PID, HEADING_PID);
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getPoseEstimate(), constraints);
    }

    public void turn(double angle) {
        double heading = getPoseEstimate().getHeading();
        RobotLog.dd(TAG, "turn: current heading "+Double.toString(heading)+", to turn angle "+Double.toString(angle));
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

    public void turnSync(double angle) {
        turn(angle);
        waitForIdle();
    }

    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectorySync(Trajectory trajectory) {
        followTrajectory(trajectory);
        waitForIdle();
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
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError", lastError.getHeading());

        RobotLog.dd(TAG, "update: x " + currentPose.getX());
        RobotLog.dd(TAG, "y " + currentPose.getY());
        RobotLog.dd(TAG, "heading " + Double.toString(currentPose.getHeading()));

        RobotLog.dd(TAG, "xError " + lastError.getX());
        RobotLog.dd(TAG, "yError " + lastError.getY());
        RobotLog.dd(TAG, "headingError "  + lastError.getHeading());

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                double correction = turnController.update(currentPose.getHeading(), targetOmega);
                RobotLog.dd(TAG, "TURN: targetOmega "+Double.toString(targetOmega)+" targetAlpha "+Double.toString(targetAlpha));
                RobotLog.dd(TAG, "correction "+Double.toString(correction));
                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

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

                fieldOverlay.setStroke("#F44336");
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

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
    public abstract List<Double> getMotorPowers(List<DcMotorEx> motors);
    public abstract void setBrakeonZeroPower(boolean flag);

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    public List<Double> getWheelVelocities() {
        List<Double> positions = getWheelPositions();
        double currentTimestamp = clock.seconds();

        List<Double> velocities = new ArrayList<>(positions.size());;
        if (lastWheelPositions != null) {
            double dt = currentTimestamp - lastTimestamp;
            for (int i = 0; i < positions.size(); i++) {
                velocities.add((positions.get(i) - lastWheelPositions.get(i)) / dt);
            }
        } else {
            for (int i = 0; i < positions.size(); i++) {
                velocities.add(0.0);
            }
        }
        String t="";
        for (int i = 0; i < lastWheelPositions.size(); i ++)
        {
            t=t+Double.toString(lastWheelPositions.get(i)) + "\t";
        }
        RobotLog.dd(TAG,"last ts: "+Double.toString(lastTimestamp)+" last wheel position: "+t);
        t="";
        for (int i = 0; i < positions.size(); i ++)
        {
            t=t+Double.toString(positions.get(i)) + "\t";
        }
        RobotLog.dd(TAG,"current ts: "+Double.toString(currentTimestamp)+" current wheel position: "+t);
        t="";
        for (int i = 0; i < velocities.size(); i ++)
        {
            t=t+Double.toString(velocities.get(i)) + "\t";
        }
        RobotLog.dd(TAG, "velocity: "+t);

        lastTimestamp = currentTimestamp;
        lastWheelPositions = positions;

        return velocities;
    }

    public abstract PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode);

    public abstract void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients);

    public void print_list_double(List<Double> list){
        //motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        for (int i = 0; i < list.size(); i ++)
        {
            String wheel_name="";
            if (i==0)
                wheel_name = "leftFront";
            else if (i==1)
                wheel_name = "leftRear";
            else if (i==2)
                wheel_name = "rightRear";
            else if (i==3)
                wheel_name = "rightFront";
            else
                wheel_name = "unexpected wheel name";

            RobotLog.dd(TAG, wheel_name+"  " +Double.toString(list.get(i)));
        }
    }

}
