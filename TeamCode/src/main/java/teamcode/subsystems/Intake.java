
package teamcode.subsystems;

import TrcCommonLib.trclib.TrcEvent;
import ftclib.FtcDcMotor;
import teamcode.RobotParams;

public class Intake
{
    private final FtcDcMotor intakeMotor;
//    private final FtcDistanceSensor intakeSensor;
//    private int pixelCount = 0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     */
    public Intake(String instanceName)
    {
        intakeMotor = new FtcDcMotor(instanceName + ".motor");
        intakeMotor.setMotorInverted(RobotParams.INTAKE_MOTOR_INVERTED);
//        if (RobotParams.Preferences.hasIntakeSensor)
//        {
//            intakeSensor = new FtcDistanceSensor(instanceName + ".sensor");
//            TrcTriggerThresholdZones analogTrigger = new TrcTriggerThresholdZones(
//                instanceName + ".analogTrigger", this::getDistance, new double[]{RobotParams.INTAKE_SENSOR_THRESHOLD},
//                false);
//        }
//        else
//        {
//            intakeSensor = null;
//        }
    }   //Intake

    /**
     * This method returns the intake motor object.
     *
     * @return intake motor object.
     */
    public FtcDcMotor getIntakeMotor()
    {
        return intakeMotor;
    }   //getIntakeMotor

    public void stop()
    {
        intakeMotor.stop();
    }   //stop

    public void setOn(double delay, double power, double duration, TrcEvent event)
    {
        intakeMotor.setPower(delay, power, duration, event);
    }   //setOn

    public void setOn(double delay, double duration, TrcEvent event)
    {
        intakeMotor.setPower(delay, RobotParams.INTAKE_FORWARD_POWER, duration, event);
    }   //setOn

    public void setOn(boolean on)
    {
        intakeMotor.setPower(on? RobotParams.INTAKE_FORWARD_POWER: 0.0);
    }   //setOn

    public void setReverse(double delay, double duration, TrcEvent event)
    {
        intakeMotor.setPower(delay, RobotParams.INTAKE_REVERSE_POWER, duration, event);
    }   //setReverse

    public void setReverse(boolean on)
    {
        intakeMotor.setPower(on? RobotParams.INTAKE_REVERSE_POWER: 0.0);
    }   //setReverse

//    /**
//     * This method is called when an analog sensor threshold has been crossed.
//     *
//     * @param context specifies the callback context.
//     */
//    private void analogTriggerCallback(Object context)
//    {
//        TrcTriggerThresholdZones.CallbackContext callbackContext = (TrcTriggerThresholdZones.CallbackContext) context;
//
//        if (msgTracer != null)
//        {
//            msgTracer.traceInfo(
//                instanceName, "Zone=%d->%d, value=%.3f",
//                callbackContext.prevZone, callbackContext.currZone, callbackContext.sensorValue);
//        }
//
//        if (callbackContext.prevZone == 1 && callbackContext.currZone == 0)
//        {
//            pixelCount++;
//            if (pixelCount >= 2)
//            {
//                intakeMotor.setPower(0.0, RobotParams.INTAKE_REVERSE_POWER, 0.5);
//            }
//        }
//    }   //analogTriggerCallback
//
//    /**
//     * This method is called the TrcTriggerThresholdZones to get the sensor data.
//     *
//     * @return distance to detected object in inches.
//     */
//    private double getDistance()
//    {
//        TrcSensor.SensorData<Double> data =
//            intakeSensor != null? intakeSensor.getProcessedData(0, FtcDistanceSensor.DataType.DISTANCE_INCH): null;
//        return data != null && data.value != null? data.value: 0.0;
//    }   //getDistance

}   //class Intake
