
package TrcCommonLib.trclib;

/**
 * This class implements the Infinite Impulse Response filter. It is useful for filtering noise from the sensor data.
 */
public class TrcIIRFilter extends TrcFilter
{
    private static final double DEF_WEIGHT = 0.9;
    private final double weight;
    private double filteredData;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param weight specifies the weight of the current data point.
     */
    public TrcIIRFilter(String instanceName, double weight)
    {
        super(instanceName);

        if (weight < 0.0 || weight > 1.0)
        {
            throw new IllegalArgumentException("Weight must be a positive fraction within 1.0.");
        }

        this.weight = weight;
        reset();
    }   //TrcIIRFilter

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcIIRFilter(String instanceName)
    {
        this(instanceName, DEF_WEIGHT);
    }   //TrcIIRFilter

    //
    // Implements TrcFilter abstract methods.
    //

    /**
     * This method resets the filter.
     */
    @Override
    public void reset()
    {
        filteredData = 0.0;
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
        return filteredData*(1.0 - weight) + data*weight;
    }   //filterData

}   //class TrcIIRFilter
