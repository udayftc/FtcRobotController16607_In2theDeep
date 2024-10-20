package teamcode.midnight.creators;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This class is used to store configuration information of all the motors used in the robot.
 *
 * Total 7 DC motors are configured.
 * 4 DC motors are used to control chassis.
 * 1 DC motor is used to control .
 * 1 DC motor is used to control .
 * 1 DC motor is used to control .
 *
 * Total 3 Servo motors are configure
 * 1 servo motor is used to control .
 * 1 servo motor is used to control .
 * 1 Servo motor is used to control .
 *
 */
public class RobotHardwareConfigurator {

    //DC motors used in chassis
    private DcMotor frontLeftDC;
    private DcMotor frontRightDC;
    private DcMotor backLeftDC;
    private DcMotor backRightDC;

    //Other DC motors used
    private DcMotor armDC;
    private DcMotor hookDC;
    private DcMotor ladderLiftDC;

    //Servo motors used
    private Servo armServo;
    private Servo droneServo;
    private Servo intakeServo;

    private Limelight3A limelight;
    private NavxMicroNavigationSensor navxMicro;
    private IntegratingGyroscope gyro;


    /* Initialize the hardware variables. Note that the strings used here as parameters
     * which must should correspond to the names assigned during the robot configuration
     * step (using the FTC Robot Controller app on the Device).
     */
    public void configureRobotHardware(HardwareMap hardwareMap) {

        //chassis_DC_motor - 4, Control Hub- Port ?
        frontRightDC =  hardwareMap.dcMotor.get("Frontleft");    //chassis_DC_motor - 1, Control Hub- Port ?
        frontLeftDC = hardwareMap.dcMotor.get("Frontright");     //chassis_DC_motor - 2, Control Hub- Port ?
        backLeftDC =   hardwareMap.dcMotor.get("Backleft");      //chassis_DC_motor - 3, Control Hub- Port ?
        backRightDC =  hardwareMap.dcMotor.get("Backright");     //chassis_DC_motor - 4, Control Hub- Port ?

        //channel_DC_motor - ?, Control Hub- Port ?
        armDC = hardwareMap.dcMotor.get("arm");                    //arm_DC_motor - 5
        ladderLiftDC = hardwareMap.dcMotor.get("LadderLift");      //ladder_DC_motor - 6
        hookDC = hardwareMap.dcMotor.get("Hook");                  //hook_DC_motor - 7

        armServo = hardwareMap.servo.get("Servoarm");              //Servoarm - 1
        droneServo = hardwareMap.servo.get("Drone");               //Drone - 2
        intakeServo = hardwareMap.servo.get("Intake");             //Intake - 3

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");

        //gyro = (IntegratingGyroscope) navxMicro; This init needs a method call, as it is taking time for Gyro Calibrating.

    }

    public DcMotor getFrontRightDC() {
        return frontRightDC;
    }

    public DcMotor getFrontLeftDC() {
        return frontLeftDC;
    }

    public DcMotor getBackLeftDC() {
        return backLeftDC;
    }

    public DcMotor getBackRightDC() {
        return backRightDC;
    }

    public DcMotor getArmDC() {
        return armDC;
    }

    public DcMotor getLadderLiftDC() {
        return ladderLiftDC;
    }

    public DcMotor getHookDC() {
        return hookDC;
    }

    public Servo getIntakeServo() {
        return intakeServo;
    }

    public Servo getDroneServo() {
        return droneServo;
    }

    public Servo getArmServo() {
        return armServo;
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    public NavxMicroNavigationSensor getNavxMicro() {
        return navxMicro;
    }

    public IntegratingGyroscope getGyro() {
        return gyro;
    }
}
