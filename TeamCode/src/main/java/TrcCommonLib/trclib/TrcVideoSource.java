
package TrcCommonLib.trclib;

/**
 * This interface provides methods to get a video frame from a video source and render a video frame to a display
 * surface.
 *
 * @param <O> specifies the type of the video frame.
 */
public interface TrcVideoSource<O>
{
    /**
     * This method takes a snapshot of the video frame.
     *
     * @param frame specifies the frame buffer to hold the video snapshot.
     * @return true if successful, false otherwise.
     */
    boolean getFrame(O frame);

    /**
     * This method displays a frame buffer to the display surface.
     *
     * @param frame specifies the video frame to be displayed.
     */
    void putFrame(O frame);

}   //interface TrcVideoSource
