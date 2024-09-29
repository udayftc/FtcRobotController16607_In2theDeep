
package TrcCommonLib.trclib;

/**
 * This class is intended to be extended by a platform dependent Tank Motion Profile Follower. The extended class
 * will implement the necessary abstract methods to start following the provided motion profile. In the future,
 * this class will also implement the motion profile following logic for motor controllers that do not have
 * native motion profile support. In this case, the extended class will provide methods to receive each profile
 * way point and program the motor controller accordingly.
 */
public abstract class TrcTankMotionProfileFollower
{
    protected final String instanceName;

    /**
     * This method starts following the supplied motion profile.
     *
     * @param profile specifies the TrcTankMotionProfile object representing the path to follow.
     *                Remember to match units!
     * @param event specifies the event to signal when path has been followed.
     * @param timeout specifies maximum number of seconds to spend following the path. 0.0 means no timeout.
     */
    public abstract void start(TrcTankMotionProfile profile, TrcEvent event, double timeout);

    /**
     * This method returns the motion profile currently being followed by the follower.
     *
     * @return profile object currently being followed (null if not following any profile).
     */
    public abstract TrcTankMotionProfile getActiveProfile();

    /**
     * This method checks if path is currently being followed.
     *
     * @return true if yes, false otherwise.
     */
    public abstract boolean isActive();

    /**
     * This method checks if path following has been cancelled.
     *
     * @return true if someone has called the cancel() method while it was running, false otherwise.
     */
    public abstract boolean isCancelled();

    /**
     * This method stops following the path and cancel the event.
     */
    public abstract void cancel();

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcTankMotionProfileFollower(String instanceName)
    {
        this.instanceName = instanceName;
    }   //TrcTankMotionProfileFollower

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
     * This method starts following the supplied motion profile.
     *
     * @param profile specifies the TrcTankMotionProfile object representing the path to follow.
     *                Remember to match units!
     * @param event specifies the event to signal when path has been followed.
     */
    public void start(TrcTankMotionProfile profile, TrcEvent event)
    {
        start(profile, event, 0.0);
    }   //start

    /**
     * This method starts following the supplied motion profile.
     *
     * @param profile specifies the TrcTankMotionProfile object representing the path to follow.
     *                Remember to match units!
     */
    public void start(TrcTankMotionProfile profile)
    {
        start(profile, null, 0.0);
    }   //start

}   //class TrcTankMotionProfileFollower
