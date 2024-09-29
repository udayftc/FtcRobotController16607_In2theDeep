
package TrcCommonLib.trclib;

/**
 * This class implements a platform independent generic analog sensor. Anything that produces analog data can use
 * this class to make itself an analog sensor that conforms to the TrcSensor class which can be used as an analog
 * trigger. To make itself an analog sensor, it inherits from the AnalogInput class and thus must provide the
 * getRawData abstract method required by TrcAnalogInput. In getRawData, it will call the AnalogDataSource
 * provided in its constructor to get the raw data.
 */
public class TrcAnalogSensor extends TrcAnalogInput
{
    /**
     * This interface is used by this class to get the analog data from the provider.
     */
    public interface AnalogDataSource
    {
        /**
         * This method returns the raw data from the analog data source.
         *
         * @return raw analog data.
         */
        Double getData();
    }   //interface AnalogDataSource

    private final AnalogDataSource dataSource;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param dataSource specifies the analog data provider.
     */
    public TrcAnalogSensor(String instanceName, AnalogDataSource dataSource)
    {
        super(instanceName, 1, 0, null);
        this.dataSource = dataSource;
    }   //TrcAnalogSensor

    /**
     * This abstract method returns the raw data with the specified index and type.
     *
     * @param index specifies the data index (not used because AnalogSensor has only one axis).
     * @param dataType specifies the data type (not used because AnalogSensor only returns raw data).
     * @return raw data from the analog data source.
     */
    public SensorData<Double> getRawData(int index, DataType dataType)
    {
        Double rawData = dataSource.getData();
        return rawData != null? new SensorData<>(TrcTimer.getCurrentTime(), rawData): null;
    }   //getRawData

}   //class TrcAnalogSensor
