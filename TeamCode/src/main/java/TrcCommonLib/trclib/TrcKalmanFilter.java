

package TrcCommonLib.trclib;

/**
 * This class implements the Kalman filter. It is useful for filtering noise from the sensor data.
 */
public class TrcKalmanFilter extends TrcFilter
{
    private static final double DEF_KQ = 0.022;
    private static final double DEF_KR = 0.617;

    private final double kQ;
    private final double kR;
    private double prevP;
    private double prevXEst;
    private boolean initialized;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param kQ specifies the KQ constant.
     * @param kR specifies the KR constant.
     */
    public TrcKalmanFilter(String instanceName, double kQ, double kR)
    {
        super(instanceName);

        this.kQ = kQ;
        this.kR = kR;
        reset();
    }   //TrcKalmanFilter

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcKalmanFilter(String instanceName)
    {
        this(instanceName, DEF_KQ, DEF_KR);
    }   //TrcKalmanFilter

    //
    // Implements TrcFilter abstract methods.
    //

    /**
     * This method resets the filter.
     */
    @Override
    public void reset()
    {
        prevP = 0.0;
        prevXEst = 0.0;
        initialized = false;
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
        if (!initialized)
        {
            prevXEst = data;
            initialized = true;
        }

        double tempP = prevP + kQ;
        double k = tempP/(tempP + kR);
        double xEst = prevXEst + k*(data - prevXEst);

        prevP = (1 - k)*tempP;
        prevXEst = xEst;

        return prevXEst;
    }   //filterData

}   //class TrcKalmanFilter
