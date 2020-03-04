package org.firstinspires.ftc.teamcode.drive.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Autonomous.FieldPosition;
import org.firstinspires.ftc.teamcode.Autonomous.Path;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.RobotLogger;
import org.firstinspires.ftc.teamcode.drive.localizer.VuforiaCamLocalizer;
import org.firstinspires.ftc.teamcode.drive.localizer.VuforiaCameraChoice;

import org.firstinspires.ftc.teamcode.TeleOp.TeleopConstants;
import org.firstinspires.ftc.teamcode.util.AllHardwareMap;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
@Disabled
public class PathTest extends LinearOpMode {
    private Trajectory trajectory;
    private BaseTrajectoryBuilder builder, strafe_builder;
    private Pose2d current_pose;
    private String TAG = "PathTest";
    private SampleMecanumDriveBase _drive = null, _strafeDrive = null;
    private AllHardwareMap hwMap;
    private Path path;
    private FieldPosition fieldPosition = null;
    private VuforiaCamLocalizer vuLocalizer = null;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstants.updateConstantsFromProperties();
        // do we need this??? all this are HW settings to enable ARM actions;
        //hwMap.liftOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //hwMap.liftOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //hwMap.liftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //hwMap.liftOne.setDirection(DcMotorSimple.Direction.REVERSE);
        //
        int[] skystonePositions = new int[2];
        skystonePositions[0] = (int) DriveConstants.TEST_SKY_STONE_POSITION;
        RobotLogger.dd(TAG, "xml file %d", skystonePositions[0]);
        String filename = "path_blue_.xml";
        int tindex = filename.indexOf(".xml");
        RobotLogger.dd(TAG, "%d", tindex);
        filename = filename.substring(0, tindex-1) + Integer.toString(skystonePositions[0])
                + filename.substring(tindex);
        Pose2d t[] = DriveConstants.parsePathXY(filename);
        RobotLogger.dd(TAG, "XY array len: " + Integer.toString(t.length));


        if (DriveConstants.USING_BULK_READ == false) {
            _drive = new SampleMecanumDriveREV(hardwareMap, false);
            _strafeDrive = new SampleMecanumDriveREV(hardwareMap, true);
        }
        else {
            _drive = new SampleMecanumDriveREVOptimized(hardwareMap, false);
            _strafeDrive = new SampleMecanumDriveREVOptimized(hardwareMap, true);
        }

        RobotLogger.dd(TAG, "unit test for path (BLUE QUARY), ARM actions?" + Integer.toString(DriveConstants.ENABLE_ARM_ACTIONS?1:0));
        Pose2d startingPos = new Pose2d(new Vector2d(-34.752, -63.936), Math.toRadians(0));
        hwMap = new AllHardwareMap(hardwareMap);
        fieldPosition = FieldPosition.RED_QUARY;

        waitForStart();

        if (isStopRequested()) return;

        if (DriveConstants.USE_VUFORIA_LOCALIZER) {
            vuLocalizer = VuforiaCamLocalizer.getSingle_instance(hardwareMap,
                    VuforiaCameraChoice.PHONE_BACK, true);
        }
        path = new Path(hwMap, this, _drive, _strafeDrive, hardwareMap, null, telemetry);
        path.BlueQuary(skystonePositions, vuLocalizer);
        //path.BlueQuary(skystonePositions, vuLocalizer);
        RobotLogger.dd(TAG, "----------done --------------------- unit test for path (BLUE QUARY)");
    }
}
