
package TrcCommonLib.trclib;

public interface TrcDistanceSensor
{
    /**
     * This method returns the distance sensor value in inches.
     *
     * @return distance in inches.
     */
    double getDistanceInches();

    default double getDistanceMillimeters()
    {
        return getDistanceInches() * TrcUtil.MM_PER_INCH;
    } //getDistanceMillimeters

    default double getDistanceMeters()
    {
        return getDistanceInches() * TrcUtil.METERS_PER_INCH;
    } //getDistanceMeters

}   //interface TrcDistanceSensor
