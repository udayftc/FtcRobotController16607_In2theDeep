
package TrcCommonLib.trclib;

import java.util.Locale;

/**
 * This class implements a performance timer to record elapsed time and interval time of a periodic task. It is a
 * performance monitoring tool. It is especially important for PID controlled loops that the loops are executed at
 * a high enough frequency or they will oscillate wildly. It also records the min and max elapsed/interval time values
 * it has seen since the last reset.
 */
public class TrcPerformanceTimer
{
    private final String instanceName;
    private long startTime;
    private long totalElapsedTime;
    private long minElapsedTime;
    private long maxElapsedTime;
    private int elapsedCount;
    private long totalIntervalTime;
    private long minIntervalTime;
    private long maxIntervalTime;
    private int intervalCount;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the name to identify this instance of the performance timer.
     */
    public TrcPerformanceTimer(String instanceName)
    {
        this.instanceName = instanceName;
        reset();
    }   //TrcPerformanceTimer

    /**
     * This method returns the performance data in string form.
     *
     * @return performance data in string form.
     */
    @Override
    public String toString()
    {
        return String.format(
            Locale.US,
            "%s: avgElapsed=.6f, minElapsed=%.6f, maxElapsed=%.6f, avgInterval=%.6f, minInterval=%.6f, " +
            "maxInterval=%.6f",
            instanceName, getAverageElapsedTime(), getMinElapsedTime(), getMaxElapsedTime(), getAverageIntervalTime(),
            getMinIntervalTime(), getMaxIntervalTime());
    }   //toString

    /**
     * This method resets the performance data.
     */
    public synchronized void reset()
    {
        startTime = 0L;
        totalElapsedTime = totalIntervalTime = 0L;
        minElapsedTime = minIntervalTime = Long.MAX_VALUE;
        maxElapsedTime = maxIntervalTime = 0L;
        elapsedCount = intervalCount = 0;
    }   //reset

    /**
     * This method is called to record the start time. It also records the interval time since the last call to
     * recordStartTime. It also checks if the interval time is the minimum or maximum it has seen since last reset.
     */
    public synchronized void recordStartTime()
    {
        long currNanoTime = TrcTimer.getNanoTime();

        if (startTime > 0L)
        {
            long intervalTime = currNanoTime - startTime;

            totalIntervalTime += intervalTime;
            intervalCount++;

            if (intervalTime < minIntervalTime)
            {
                minIntervalTime = intervalTime;
            }

            if (intervalTime > maxIntervalTime)
            {
                maxIntervalTime = intervalTime;
            }
        }

        startTime = currNanoTime;
    }   //recordStartTime

    /**
     * This method is called to record the elapsed time since the last start time. It also checks if the elapsed time
     * is the minimum or maximum it has seen since last reset.
     */
    public synchronized void recordEndTime()
    {
        // PerformanceTimer could have been enabled after the recordStartTime call, so it will miss recording startTime.
        // In this case, skip recordEndTime since we don't have a valid startTime.
        if (startTime > 0L)
        {
            long elapsedTime = TrcTimer.getNanoTime() - startTime;

            totalElapsedTime += elapsedTime;
            elapsedCount++;

            if (elapsedTime < minElapsedTime)
            {
                minElapsedTime = elapsedTime;
            }

            if (elapsedTime > maxElapsedTime)
            {
                maxElapsedTime = elapsedTime;
            }
        }
    }   //recordEndTime

    /**
     * This method calculates the average elapsed time so far.
     *
     * @return average elapsed time in seconds.
     */
    public synchronized double getAverageElapsedTime()
    {
        return (totalElapsedTime / 1000000000.0) / elapsedCount;
    }   //getAverageElapsedTime

    /**
     * This method returns the minimum elapsed time since the last reset.
     *
     * @return minimum elapsed time in seconds.
     */
    public synchronized double getMinElapsedTime()
    {
        return minElapsedTime / 1000000000.0;
    }   //getMinElapsedTime

    /**
     * This method returns the maximum elapsed time since the last reset.
     *
     * @return maximum elapsed time in seconds.
     */
    public synchronized double getMaxElapsedTime()
    {
        return maxElapsedTime / 1000000000.0;
    }   //getMaxElapsedTime

    /**
     * This method calculates the average interval time so far.
     *
     * @return average interval time in seconds.
     */
    public synchronized double getAverageIntervalTime()
    {
        return (totalIntervalTime / 1000000000.0) / intervalCount;
    }   //getAverageIntervalTime

    /**
     * This method returns the minimum interval time since the last reset.
     *
     * @return minimum interval time in seconds.
     */
    public synchronized double getMinIntervalTime()
    {
        return minIntervalTime / 1000000000.0;
    }   //getMinIntervalTime

    /**
     * This method returns the maximum interval time since the last reset.
     *
     * @return maximum interval time in seconds.
     */
    public synchronized double getMaxIntervalTime()
    {
        return maxIntervalTime / 1000000000.0;
    }   //getMaxIntervalTime

}   //class TrcPerformanceTimer
