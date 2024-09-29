
package TrcCommonLib.trclib;

/**
 * This class implements the Spurious Data filter. It is used for detecting and discarding bogus sensor data.
 * When spurious data is detected, it is discarded and the previous data is returned.
 */
public class TrcSpuriousFilter extends TrcFilter
{
    private final double distanceThreshold;
    private Double prevData;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param distanceThreshold specifies the distance threshold from previous data point to be considered spurious.
     */
    public TrcSpuriousFilter(String instanceName, double distanceThreshold)
    {
        super(instanceName);

        this.distanceThreshold = distanceThreshold;
        reset();
    }   //TrcSpuriousFilter

    //
    // Implements TrcFilter abstract methods.
    //

    /**
     * This method resets the filter.
     */
    @Override
    public void reset()
    {
        prevData = null;
    }   //reset

    /**
     * This method returns the filtered data.
     *
     * @param data specifies the data value to be filtered.
     * @return filtered data.
     */
    @Override
    public double filterData(double data)
    {
        if (prevData != null && Math.abs(data - prevData) >= distanceThreshold)
        {
            tracer.traceWarn(instanceName, "Spurious data detected (data=%f, prev=%f)", data, prevData);
            data = prevData;
        }
        else
        {
            prevData = data;
        }

        return data;
    }   //filterData

}   //class TrcSpuriousFilter
