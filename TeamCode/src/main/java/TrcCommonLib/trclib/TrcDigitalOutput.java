
package TrcCommonLib.trclib;

/**
 * This class implements a platform independent Digital Output port device.
 */
public abstract class TrcDigitalOutput
{
    private static final String moduleName = TrcDigitalOutput.class.getSimpleName();
    protected static TrcElapsedTimer setOutputElapsedTimer = null;

    /**
     * This method is provided by the platform dependent digital output port device to set the state of the output
     * port.
     *
     * @param state specifies state of the output port.
     */
    public abstract void setState(boolean state);

    private final String instanceName;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcDigitalOutput(String instanceName)
    {
        this.instanceName = instanceName;
    }   //TrcDigitalOutput

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method enables/disables the elapsed timers for performance monitoring.
     *
     * @param enabled specifies true to enable elapsed timers, false to disable.
     */
    public static void setElapsedTimerEnabled(boolean enabled)
    {
        if (enabled)
        {
            if (setOutputElapsedTimer == null)
            {
                setOutputElapsedTimer = new TrcElapsedTimer(moduleName + ".setOutput", 2.0);
            }
        }
        else
        {
            setOutputElapsedTimer = null;
        }
    }   //setElapsedTimerEnabled

    /**
     * This method prints the elapsed time info using the given tracer.
     *
     * @param tracer specifies the tracer to be used to print the info.
     */
    public static void printElapsedTime(TrcDbgTrace tracer)
    {
        if (setOutputElapsedTimer != null)
        {
            setOutputElapsedTimer.printElapsedTime(tracer);
        }
    }   //printElapsedTime

}   //class TrcDigitalOutput
