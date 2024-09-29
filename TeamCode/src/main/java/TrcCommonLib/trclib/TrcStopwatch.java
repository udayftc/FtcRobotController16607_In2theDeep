
package TrcCommonLib.trclib;

/**
 * This class implements a stopwatch.
 */
public class TrcStopwatch
{
    private double startTime = 0.0;
    private double lastElapsedTime = 0.0;

    /**
     * This method starts the stopwatch.
     */
    public synchronized void start()
    {
        lastElapsedTime = 0.0;
        startTime = TrcTimer.getCurrentTime();
    }   //start

    /**
     * This method stops the stopwatch.
     */
    public synchronized void stop()
    {
        if (isRunning())
        {
            lastElapsedTime = TrcTimer.getCurrentTime() - startTime;
            startTime = 0.0;
        }
    }   //stop

    /**
     * This method checks if the stopwatch is running.
     *
     * @return true if the stopwatch is running, false otherwise.
     */
    public synchronized boolean isRunning()
    {
        return startTime != 0.0;
    }   //isRunning

    /**
     * This method returns the elapsed time since the start time. If the stopwatch is not running, it returns the
     * last elapsed time.
     *
     * @return elapsed time since start, -1 if the stopwatch wasn't started.
     */
    public synchronized double getElapsedTime()
    {
        return isRunning() ? TrcTimer.getCurrentTime() - startTime : lastElapsedTime;
    }   //getElapsedTime;

}   //class TrcStopwatch
