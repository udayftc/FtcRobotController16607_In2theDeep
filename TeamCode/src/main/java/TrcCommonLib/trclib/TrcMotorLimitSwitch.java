
package TrcCommonLib.trclib;

/**
 * This class implements a platform independent digital input sensor extending TrcDigitalInput. It provides
 * implementation of the abstract methods in TrcDigitalInput. The digital input sensor in this case is one
 * of the motor limit switches. This allows the motor limit switch to be used as a Digital Trigger
 * for operations such as auto zero calibration and limit switch notification callback.
 */
public class TrcMotorLimitSwitch extends TrcDigitalInput
{
    private TrcMotor motor;
    private boolean upperLimitSwitch = false;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor specifies the motor controller that hosted the limit switch.
     * @param upperLimitSwitch specifies true for the upper limit switch, false for lower limit switch.
     */
    public TrcMotorLimitSwitch(String instanceName, TrcMotor motor, boolean upperLimitSwitch)
    {
        super(instanceName);
        this.motor = motor;
        this.upperLimitSwitch = upperLimitSwitch;
    }   //TrcMotorLimitSwitch

    //
    // Implements TrcDigitalInput abstract methods.
    //

    /**
     * This method returns the state of the digital input sensor.
     *
     * @return true if the digital input sensor is active, false otherwise.
     */
    @Override
    public boolean getInputState()
    {
        return upperLimitSwitch? motor.isUpperLimitSwitchActive(): motor.isLowerLimitSwitchActive();
    }   //getInputState

    /**
     * This method inverts the polarity of the limit switch by configuring it to be normally open (non-inverted) or
     * normally close (inverted).
     *
     * @param inverted specifies true to invert and false otherwise.
     */
    @Override
    public void setInverted(boolean inverted)
    {
        if (upperLimitSwitch)
        {
            motor.setUpperLimitSwitchInverted(inverted);
        }
        else
        {
            motor.setLowerLimitSwitchInverted(inverted);
        }
    }   //setInverted

}   //class TrcMotorLimitSwitch
