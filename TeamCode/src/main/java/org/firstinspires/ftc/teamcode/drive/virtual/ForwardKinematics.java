package org.firstinspires.ftc.teamcode.drive.virtual;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Log;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

import java.util.List;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

// wheel to Robot
public class ForwardKinematics {
    public static Pose2d wheelToRobotVelocities(List<Double> wheelVelocities){
        RobotLogger.dd("ForwardKinematics", "wheelToRobotVelocities");
        double lateralMultiplier = 1.0;
        double k = (DriveConstants.TRACK_WIDTH + DriveConstants.WHEEL_BASE) / 2.0;
        double frontLeft = wheelVelocities.get(0);
        double rearLeft = wheelVelocities.get(1);
        double rearRight = wheelVelocities.get(2);
        double frontRight = wheelVelocities.get(3);
        return (new Pose2d(rearLeft + frontRight + frontLeft + rearRight,
                (rearLeft + frontRight - frontLeft - rearRight) / lateralMultiplier,
                (rearRight + frontRight - frontLeft - rearLeft) / k * 0.25));
    }


    public static Pose2d relativeOdometryUpdate(Pose2d fieldPose, Pose2d robotPoseDelta) {
        double dtheta = robotPoseDelta.getHeading();
        double sinTerm, cosTerm;

        if (Angle.epsilonEquals(dtheta,  0.0)) {
            sinTerm = 1.0 - dtheta * dtheta / 6.0;
            cosTerm = dtheta / 2.0;
        } else {
            sinTerm = sin(dtheta) / dtheta;
            cosTerm = (1 - cos(dtheta)) / dtheta;
        }

        Vector2d fieldPositionDelta = new Vector2d(
                sinTerm * robotPoseDelta.getX() - cosTerm * robotPoseDelta.getY(),
                cosTerm * robotPoseDelta.getX() + sinTerm * robotPoseDelta.getY()
        );

        Pose2d fieldPoseDelta = new Pose2d(fieldPositionDelta.rotated(fieldPose.getHeading()), robotPoseDelta.getHeading());
        RobotLogger.dd("ForwardKinematics", "Kinematics: relativeOdometryUpdate, fieldPoseDelta " + fieldPoseDelta.toString());

        return (new Pose2d(
                fieldPose.getX() + fieldPoseDelta.getX(),
                fieldPose.getY() + fieldPoseDelta.getY(),
                Angle.norm(fieldPose.getHeading() + fieldPoseDelta.getHeading())));

    }

}
