

package TrcCommonLib.trclib;

/**
 * This class implements a generic filter to be extended by a specific filter class. This class cannot be
 * instantiated by itself.
 */
public abstract class TrcFilter
{
    /**
     * This method resets the filter.
     */
    public abstract void reset();

    /**
     * This method returns the filtered data.
     *
     * @param data specifies the data value to be filtered.
     * @return filtered data.
     */
    public abstract double filterData(double data);

    protected final TrcDbgTrace tracer;
    protected final String instanceName;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    protected TrcFilter(String instanceName)
    {
        tracer = new TrcDbgTrace(instanceName);
        this.instanceName = instanceName;
    }   //TrcFilter

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

}   //class TrcFilter
