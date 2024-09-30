
package teamcode.subsystems;

import TrcCommonLib.trclib.TrcEvent;
import ftclib.FtcDcMotor;
import teamcode.RobotParams;

public class Intake
{
    private final FtcDcMotor intakeMotor;

    /**
     * Constructor: Creates an instance of the object.@param instanceName specifies the hardware name.
     */
    public Intake(String instanceName)
    {
        intakeMotor = new FtcDcMotor(instanceName + ".motor");
        intakeMotor.setMotorInverted(RobotParams.INTAKE_MOTOR_INVERTED);
    }   //Intake

    /**
     * This method returns the intake motor object.@return intake motor object.
     */
    public FtcDcMotor getIntakeMotor()
    {
        return intakeMotor;
    }   //getIntakeMotor

    public void stop()
    {
        intakeMotor.stop();
    }   //stop

    public void setOn(double delay, double power, double duration, TrcEvent event)
    {
        intakeMotor.setPower(delay, power, duration, event);
    }   //setOn

    public void setOn(double delay, double duration, TrcEvent event)
    {
        intakeMotor.setPower(delay, RobotParams.INTAKE_FORWARD_POWER, duration, event);
    }   //setOn

    public void setOn(boolean on)
    {
        intakeMotor.setPower(on? RobotParams.INTAKE_FORWARD_POWER: 0.0);
    }   //setOn

    public void setReverse(double delay, double duration, TrcEvent event)
    {
        intakeMotor.setPower(delay, RobotParams.INTAKE_REVERSE_POWER, duration, event);
    }   //setReverse

    public void setReverse(boolean on)
    {
        intakeMotor.setPower(on? RobotParams.INTAKE_REVERSE_POWER: 0.0);
    }   //setReverse


}   //class Intake
