
package teamcode.drivebases;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcMecanumDriveBase;
import TrcCommonLib.trclib.TrcOdometryWheels;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPurePursuitDrive;
import teamcode.RobotParams;

/**
 * This class creates the RobotDrive subsystem that consists of wheel motors and related objects for driving the
 * robot.
 */
public class MecanumDrive extends RobotDrive
{
    /**
     * Constructor: Create an instance of the object.
     */
    public MecanumDrive()
    {
        super();
        driveMotors = createDriveMotors(driveMotorNames, driveMotorInverted);
        driveBase = new TrcMecanumDriveBase(
            driveMotors[INDEX_LEFT_FRONT], driveMotors[INDEX_LEFT_BACK],
            driveMotors[INDEX_RIGHT_FRONT], driveMotors[INDEX_RIGHT_BACK], gyro);
        if (RobotParams.Preferences.useExternalOdometry)
        {
            // Create the external odometry device that uses the right back encoder port as the X odometry and
            // the left and right front encoder ports as the Y1 and Y2 odometry. Gyro will serve as the angle
            // odometry.
            TrcOdometryWheels driveBaseOdometry = new TrcOdometryWheels(
                new TrcOdometryWheels.AxisSensor(
                    driveMotors[INDEX_RIGHT_BACK], RobotParams.X_ODWHEEL_Y_OFFSET, RobotParams.X_ODWHEEL_X_OFFSET),
                new TrcOdometryWheels.AxisSensor[] {
                    new TrcOdometryWheels.AxisSensor(
                        driveMotors[INDEX_LEFT_FRONT], RobotParams.YLEFT_ODWHEEL_X_OFFSET,
                        RobotParams.YLEFT_ODWHEEL_Y_OFFSET),
                    new TrcOdometryWheels.AxisSensor(
                        driveMotors[INDEX_RIGHT_FRONT], RobotParams.YRIGHT_ODWHEEL_X_OFFSET,
                        RobotParams.YRIGHT_ODWHEEL_Y_OFFSET)},
                gyro);
            // Set the drive base to use the external odometry device overriding the built-in one.
            driveBase.setDriveBaseOdometry(driveBaseOdometry);
            driveBase.setOdometryScales(RobotParams.ODWHEEL_INCHES_PER_COUNT, RobotParams.ODWHEEL_INCHES_PER_COUNT);
        }
        else
        {
            driveBase.setOdometryScales(RobotParams.XPOS_INCHES_PER_COUNT, RobotParams.YPOS_INCHES_PER_COUNT);
        }
        //
        // Create and initialize PID controllers.
        //
        TrcDbgTrace tracer = TrcDbgTrace.getGlobalTracer();
        TrcPidController.PidParameters xPosPidParams = new TrcPidController.PidParameters(
            RobotParams.xPosPidCoeff, RobotParams.XPOS_TOLERANCE, driveBase::getXPosition);
        TrcPidController.PidParameters yPosPidParams = new TrcPidController.PidParameters(
            RobotParams.yPosPidCoeff, RobotParams.YPOS_TOLERANCE, driveBase::getYPosition);
        TrcPidController.PidParameters turnPidParams = new TrcPidController.PidParameters(
            RobotParams.turnPidCoeff, RobotParams.TURN_TOLERANCE, driveBase::getHeading);

        pidDrive = new TrcPidDrive("pidDrive", driveBase, xPosPidParams, yPosPidParams, turnPidParams);
        pidDrive.setStallDetectionEnabled(true);
        pidDrive.getXPidCtrl().setRampRate(RobotParams.X_RAMP_RATE);
        pidDrive.getYPidCtrl().setRampRate(RobotParams.Y_RAMP_RATE);
        pidDrive.getTurnPidCtrl().setRampRate(RobotParams.TURN_RAMP_RATE);
        pidDrive.getTurnPidCtrl().setAbsoluteSetPoint(true);
        // FTC robots generally have USB performance issues where the sampling rate of the gyro is not high enough.
        // If the robot turns too fast, PID will cause oscillation. By limiting turn power, the robot turns slower.
        pidDrive.getTurnPidCtrl().setOutputLimit(RobotParams.TURN_POWER_LIMIT);

        // AbsoluteTargetMode eliminates cumulative errors on multi-segment runs because drive base is keeping track
        // of the absolute target position.
        pidDrive.setAbsoluteTargetModeEnabled(true);
        pidDrive.setTraceLevel(TrcDbgTrace.MsgLevel.INFO, false, false, false);

        purePursuitDrive = new TrcPurePursuitDrive(
            "purePursuitDrive", driveBase,
            RobotParams.PPD_FOLLOWING_DISTANCE, RobotParams.PPD_POS_TOLERANCE, RobotParams.PPD_TURN_TOLERANCE,
            RobotParams.xPosPidCoeff, RobotParams.yPosPidCoeff, RobotParams.turnPidCoeff, RobotParams.velPidCoeff);
        purePursuitDrive.setStallDetectionEnabled(true);
        purePursuitDrive.setFastModeEnabled(true);
        purePursuitDrive.setTraceLevel(TrcDbgTrace.MsgLevel.INFO, false, false, false);
    }   //MecanumDrive

}   //class MecanumDrive
