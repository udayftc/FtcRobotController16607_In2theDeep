
package TrcCommonLib.trclib;

/**
 * This class implements a generic warp space. A warp space is originally a linear space with two points: a low
 * point A and a high point B. The space is warped such that point A and point B represent the same physical position
 * in space even though the linear distance between the two points is far away. A typical example of warp space is
 * demonstrated by a compass. A compass has a low point of 0-degree and a high point of 360-degree but these two
 * points are at the same physical position (NORTH).
 */
public class TrcWarpSpace
{
    private final String instanceName;
    private final double warpSpaceRange;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param warpSpaceLowPoint specifies the low point of the warp range.
     * @param warpSpaceHighPoint specifies the high point of the warp range.
     */
    public TrcWarpSpace(String instanceName, double warpSpaceLowPoint, double warpSpaceHighPoint)
    {
        if (warpSpaceHighPoint > warpSpaceLowPoint)
        {
            this.instanceName = instanceName;
            this.warpSpaceRange = warpSpaceHighPoint - warpSpaceLowPoint;
        }
        else
        {
            throw new IllegalArgumentException("HighPoint must be greater than LowPoint.");
        }
    }   //TrcWarpSpace

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method returns the optimized target position such that the travel distance to the target is minimum.
     * In the compass example, if one is currently headed NORTH (0-degree) and wants to turn to WEST (270-degree),
     * one may turn all the way 270 degrees clockwise and landed pointing WEST. But one could also turn towards the
     * warp point (0-degree), passed it and landed WEST by turning counter clockwise 90 degrees.
     *
     * @param targetPos specifies the target position.
     * @param currentPos specifies the current position.
     * @param range specifies the warp space range.
     * @return optimized target position resulted in shorter traveling distance.
     */
    public static double getOptimizedTarget(double targetPos, double currentPos, double range)
    {
        double distance = (targetPos - currentPos) % range;
        double absDistance = Math.abs(distance);

        return currentPos + ((absDistance > (range / 2)) ? -Math.signum(distance) * (range - absDistance) : distance);
    }   //getOptimizedTarget

    /**
     * This method returns the optimized target position such that the travel distance to the target is minimum.
     * In the compass example, if one is currently headed NORTH (0-degree) and wants to turn to WEST (270-degree),
     * one may turn all the way 270 degrees clockwise and landed pointing WEST. But one could also turn towards the
     * warp point (0-degree), passed it and landed WEST by turning counter clockwise 90 degrees.
     *
     * @param targetPos specifies the target position.
     * @param currentPos specifies the current position.
     * @return optimized target position resulted in shorter traveling distance.
     */
    public double getOptimizedTarget(double targetPos, double currentPos)
    {
        return getOptimizedTarget(targetPos, currentPos, warpSpaceRange);
    }   //getOptimizedTarget

}   //class TrcWarpSpace
