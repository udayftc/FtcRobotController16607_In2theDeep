
package TrcCommonLib.trclib;

/**
 * This class implements Performance Metrics for Vision. It keeps track of the average time for vision to process a
 * frame as well as the process frame rate.
 */
public class TrcVisionPerformanceMetrics
{
    private final String instanceName;
    private final TrcDbgTrace tracer;
    private double startTime = 0.0;
    private double totalProcessedTime = 0.0;
    private long totalProcessedFrames = 0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param tracer specifies the tracer to be used to print performance info.
     */
    public TrcVisionPerformanceMetrics(String instanceName, TrcDbgTrace tracer)
    {
        this.instanceName = instanceName;
        this.tracer = tracer;
    }   //TrcVisionPerformanceMetrics

    /**
     * This method resets the pipeline performance metrics. It is typically called before enabling the pipeline.
     */
    public void reset()
    {
        startTime = TrcTimer.getCurrentTime();
        totalProcessedTime = 0.0;
        totalProcessedFrames = 0;
    }   //reset

    /**
     * This method is called to log the processing time of the pipeline.
     *
     * @param startTime specifies the timestamp when the processing starts.
     */
    public void logProcessingTime(double startTime)
    {
        totalProcessedTime += TrcTimer.getCurrentTime() - startTime;
        totalProcessedFrames++;
    }   //logProcessingTime

    /**
     * This method prints the pipeline performance metrics using the given tracer.
     */
    public void printMetrics()
    {
        tracer.traceInfo(
            instanceName, "AvgProcessTime=%.6f, FrameRate=%f",
            totalProcessedTime/totalProcessedFrames, totalProcessedFrames/(TrcTimer.getCurrentTime() - startTime));
    }   //printMetrics

}   //class TrcVisionPerformanceMetrics
