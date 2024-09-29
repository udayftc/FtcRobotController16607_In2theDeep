
package TrcCommonLib.trclib;

/**
 * This interface provides methods to grab image from the video input, render image to video output and detect
 * objects from the acquired image.
 *
 * @param <I> specifies the type of the input image.
 * @param <O> specifies the type of the detected object.
 */
public interface TrcVisionProcessor<I, O> extends TrcVideoSource<I>
{
    /**
     * This method is called to detect objects in the acquired image frame.
     *
     * @param image specifies the image to be processed.
     * @return detected objects, null if none detected.
     */
    O[] processFrame(I image);

    /**
     * This method returns the selected intermediate output Mat to be displayed.
     *
     * @return selected output mat.
     */
    I getSelectedOutput();

}   //interface TrcVisionProcessor
