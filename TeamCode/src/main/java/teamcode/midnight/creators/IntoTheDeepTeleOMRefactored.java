package teamcode.midnight.creators;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

// Into the deep main Teleop mode 16607
@TeleOp(name = "Robot Manual Drive", group = "Linear Opmode")
public class IntoTheDeepTeleOMRefactored extends LinearOpMode {

    /* Declare OpMode members */
    RobotHardwareConfigurator myRobotHW = new RobotHardwareConfigurator();

    //TBD private DistanceSensor sensorColorRange;

    private DcMotor frontLeftDC;
    private DcMotor frontRightDC;
    private DcMotor backLeftDC;
    private DcMotor backRightDC;

    private DcMotor armDC;
    private DcMotor hookDC;
    private DcMotor ladderLiftDC;

    private Servo armServo;
    private Servo droneServo;
    private Servo intakeServo;

/*  Code Refactored - Use RobotHardwareConfigurator file instead
    private DcMotor Frontright;
    private DcMotor Frontleft;
    private DcMotor Backleft;
    private DcMotor Backright;

    private DcMotor LadderLift;
    private DcMotor Hook;
    private DcMotor arm;

    private Servo Intake;
    private Servo Drone;
    private Servo Servoarm;
    */

    private Limelight3A limelight;
    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;

    /* Code Refactored - Use Constant file instead
    private int armhighpos = 450;
    private int armlowpos = 30;
    private int ladderhighpos = 1900;
    private int ladderlowpos = 0;
	private int laddermidpos = 850;
    private int hookhighpos = 1800;
    private int hooklowpos = 40;*/
    //private int tgtposition = 0;

    private int flencoderpos = 0;
    private int frencoderpos = 0;
    private int blencoderpos = 0;
    private int brencoderpos = 0;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "HW Configuration Mapping Initialized");
        myRobotHW.configureRobotHardware(hardwareMap);
        telemetry.update();

        telemetry.addData("Status", "HW Configuration Initialized");
        init_motors();
        telemetry.update();

        telemetry.addData("Status", "Actuators Initialized");
        init_navx();
        init_LL();
        telemetry.update();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        waitForStart();
        telemetry.log().clear();

        while (opModeIsActive()) {
            float yawDegrees = run_navx();
            run_LL();
            telemetry.addData("yawreturned", yawDegrees);
            telemetry.update();
            Mecanumdriveyaw (yawDegrees);
            /* servo open and close and positions */ /* Hook Positions */
            if (gamepad1.dpad_left) {
                intakeServo.setPosition(0);
            } if (gamepad1.dpad_right) {
                intakeServo.setPosition(1);
            } if (gamepad1.left_bumper) {
                armServo.setPosition(0);
            } if (gamepad1.right_bumper) {
                armServo.setPosition(1);
            } if (gamepad1.dpad_up) {
                armDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm_run_to_position(DcMotorConstant.ARM_HIGH_POS);
            } if (gamepad1.dpad_down) {
                arm_run_to_position(DcMotorConstant.ARM_LOW_POS);
            }

            /* LadderLift positions */
            if (gamepad2.a) {
                ladder_run_to_position(DcMotorConstant.LADDER_LOW_POS);
            } if (gamepad2.x) {
                ladder_run_to_position(DcMotorConstant.LADDER_MID_POS);
            } if (gamepad2.y) {
                ladder_run_to_position(DcMotorConstant.LADDER_HIGH_POS);
            } if (gamepad2.dpad_up) {
                hook_run_to_position(DcMotorConstant.HOOK_HIGH_POS);
            } if (gamepad2.dpad_down) {
                hook_run_to_position(DcMotorConstant.HOOK_LOW_POS);
                hookDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            telemetry.update();
            idle();
        }
        limelight.stop();
    }

    public void init_LL() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        /*Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.*/
        limelight.start();
    }
    public void init_navx() throws InterruptedException {
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope) navxMicro;
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        // Wait until the gyro calibration is complete
        timer.reset();
        while (navxMicro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            /*noinspection BusyWait */
            Thread.sleep(50);
        }
        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated.");
        telemetry.clear();
        telemetry.update();
    }
    public void init_motors() {
        initiateChassisDCMotors();
        initiateOtherDCMotors();
        initiateServoMotors();
        telemetry.update();
    }

    private void initiateChassisDCMotors() {
        telemetry.addData("DcMotor initiateChassisDCMotors:", "Configuration Started");

        backLeftDC = myRobotHW.getBackLeftDC();
        backRightDC = myRobotHW.getBackRightDC();
        frontLeftDC = myRobotHW.getFrontLeftDC();
        frontRightDC = myRobotHW.getFrontRightDC();

        //Refactored Backleft = hardwareMap.get(DcMotor.class, "Backleft");
        //Refactored Backright = hardwareMap.get(DcMotor.class, "Backright");
        //Refactored Frontleft = hardwareMap.get(DcMotor.class, "Frontleft");
        //Refactored Frontright = hardwareMap.get(DcMotor.class, "Frontright");

        backLeftDC.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionBL", backLeftDC.getDirection());
        backLeftDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeBL:", backLeftDC.getCurrentPosition());
        backLeftDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRightDC.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionBR", backRightDC.getDirection());
        backRightDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeBR:", backRightDC.getCurrentPosition());
        backRightDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftDC.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionFL", frontLeftDC.getDirection());
        frontLeftDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeFL:", frontLeftDC.getCurrentPosition());
        frontLeftDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRightDC.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionFR", frontRightDC.getDirection());
        frontRightDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeFR:", frontRightDC.getCurrentPosition());
        frontRightDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("DcMotor initiateChassisDCMotors:", "Configuration Completed");
    }

    private void initiateOtherDCMotors() {
        telemetry.addData("DcMotor initiateOtherDCMotors:", "Configuration Started");

        armDC = myRobotHW.getArmDC();
        hookDC = myRobotHW.getHookDC();
        ladderLiftDC = myRobotHW.getLadderLiftDC();

        //Refactored armDC = hardwareMap.get(DcMotor.class, "arm");
        //Refactored hookDC = hardwareMap.get(DcMotor.class, "Hook");
        //Refactored ladderLiftDC = hardwareMap.get(DcMotor.class, "LadderLift");

        hookDC.setDirection(DcMotorSimple.Direction.REVERSE);
        hookDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hookDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ladderLiftDC.setDirection(DcMotorSimple.Direction.REVERSE);
        ladderLiftDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ladderLiftDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armDC.setDirection(DcMotor.Direction.FORWARD);
        armDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armDC.setPower(0.0);

        telemetry.addData("DcMotor initiateOtherDCMotors:", "Configuration Completed");
    }

    private void initiateServoMotors() {
        armServo = myRobotHW.getArmServo();
        droneServo = myRobotHW.getDroneServo();
        intakeServo = myRobotHW.getIntakeServo();

        //Refactored intakeServo = hardwareMap.get(Servo.class, "Intake");
        //Refactored armServo = hardwareMap.get(Servo.class, "Servoarm");
        //Refactored Drone = hardwareMap.get(Servo.class, "Drone");
    }

    public float run_navx() {

        AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
       /* telemetry.addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.update();*/
        return angles.firstAngle;
    }
    public void run_LL() {
        LLStatus status = limelight.getStatus();
        //telemetry.addData("Name", "%s",status.getName());
        //telemetry.addData("Pipeline", "Index: %d, Type: %s",
        //        status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result != null) {
            // Access general information
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();

            if (result.isValid()) {
                //    telemetry.addData("tx", result.getTx());
                //    telemetry.addData("txnc", result.getTxNC());
                //    telemetry.addData("ty", result.getTy());
                //    telemetry.addData("tync", result.getTyNC());

                //    telemetry.addData("Botpose", botpose.toString());

                // Access barcode results
                List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                for (LLResultTypes.BarcodeResult br : barcodeResults) {
//                    telemetry.addData("Barcode", "Data: %s", br.getData());
                }

                // Access classifier results
                List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                for (LLResultTypes.ClassifierResult cr : classifierResults) {
                    //telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                }

                // Access detector results
                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                for (LLResultTypes.DetectorResult dr : detectorResults) {
                    //telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                }

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    // telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }

                // Access color results
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    //telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                }
            }
        } else {
            //telemetry.addData("Limelight", "No data available");
        }
    }

    public void Mecanumdriveyaw(double yawDegrees) {
        frontRightDC.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDC.setDirection(DcMotorSimple.Direction.REVERSE);

        double joystickX = gamepad1.right_stick_y;  // Joystick X-axis (left/right strafing)
        double joystickY = -gamepad1.right_stick_x;  // Joystick Y-axis (forward/backward)
        double omega = -gamepad1.left_stick_x*2;  // Joystick rotation input (turning)
        double heading = Math.toRadians(yawDegrees);
        double L = 0.24;  // half of the diagonal distance between wheels
        double vx = joystickX * Math.cos(heading) - joystickY * Math.sin(heading);
        double vy = joystickX * Math.sin(heading) + joystickY * Math.cos(heading);

        double powerFL = vx + vy + omega * L;  // Front Left
        double powerFR = vx - vy - omega * L;  // Front Right
        double powerBL = vx - vy + omega * L;  // Back Left
        double powerBR = vx + vy - omega * L;  // Back Right

        // Normalize motor powers to ensure none exceed [-1, 1]
        double maxPower = Math.max(Math.max(Math.abs(powerFL), Math.abs(powerFR)),
                Math.max(Math.abs(powerBL), Math.abs(powerBR)));
        if (maxPower > 1.0) {
            powerFL /= maxPower;
            powerFR /= maxPower;
            powerBL /= maxPower;
            powerBR /= maxPower;
        }
        frontLeftDC.setPower(powerFL);
        frontRightDC.setPower(powerFR);
        backLeftDC.setPower(powerBL);
        backRightDC.setPower(powerBR);
    }

    public void ladder_run_to_position(int tgtposition) {
        ladderLiftDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ladderLiftDC.setTargetPosition(tgtposition);
        ladderLiftDC.setPower(1);
        ladderLiftDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (ladderLiftDC.isBusy()) {
            telemetry.addData("LadderPos:", ladderLiftDC.getCurrentPosition());
            telemetry.update();
        }
    }
    public void hook_run_to_position(int hooktgtpos) {
        hookDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hookDC.setTargetPosition(hooktgtpos);
        hookDC.setPower(0.6);
        hookDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (hookDC.isBusy()) {
            telemetry.addData("Hook Pos:", hookDC.getCurrentPosition());
            telemetry.update();
        }
    }
    public void arm_run_to_position(int armtgtpos) {
        armDC.setTargetPosition(armtgtpos);
        armDC.setPower(1.0);
        armDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armDC.isBusy()) {
            telemetry.addData("arm Pos:", armDC.getCurrentPosition());
            telemetry.update();
        }
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees) {
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public void Mecanumdrive() {
        frontRightDC.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDC.setDirection(DcMotorSimple.Direction.REVERSE);
        double h = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
        double robotAngle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI / 4;
        double rightX = gamepad1.left_stick_x;
        final double v1 = h * Math.sin(robotAngle) - rightX;
        final double v2 = h * Math.cos(robotAngle) + rightX;
        final double v3 = h * Math.cos(robotAngle) - rightX;
        final double v4 = h * Math.sin(robotAngle) + rightX;

        frontLeftDC.setPower(v1);
        frontRightDC.setPower(v2);
        backLeftDC.setPower(v3);
        backRightDC.setPower(v4);
    }
}



