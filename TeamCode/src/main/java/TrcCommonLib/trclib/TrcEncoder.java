
package TrcCommonLib.trclib;

/**
 * This interface specifies a common implementation of a generic encoder with which makes different types of encoders
 * compatible with each other.
 */
public interface TrcEncoder
{
    /**
     * This method resets the encoder position.
     */
    void reset();

    /**
     * This method reads the normalized absolute position of the encoder.
     *
     * @return normalized absolute position of the encoder.
     */
    double getRawPosition();

    /**
     * This method returns the encoder position adjusted by scale and offset.
     *
     * @return encoder position adjusted by scale and offset.
     */
    double getPosition();

    /**
     * This method reverses the direction of the encoder.
     *
     * @param inverted specifies true to reverse the encoder direction, false otherwise.
     */
    void setInverted(boolean inverted);

    /**
     * This method checks if the encoder direction is inverted.
     *
     * @return true if encoder direction is rerversed, false otherwise.
     */
    boolean isInverted();

    /**
     * This method sets the encoder scale and offset.
     *
     * @param scale specifies the scale value.
     * @param offset specifies the offset value.
     * @param zeroOffset specifies the zero offset for absolute encoder.
     */
    void setScaleAndOffset(double scale, double offset, double zeroOffset);

}   //interface TrcEncoder
