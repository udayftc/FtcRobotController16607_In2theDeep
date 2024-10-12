package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp
public class IntoTheDeepTeleOM extends LinearOpMode {

    //private DistanceSensor sensorColorRange;

    private DcMotor LadderLift;
    private DcMotor Hook;
    private DcMotor Frontright;
    private DcMotor Frontleft;
    private DcMotor Backleft;
    private DcMotor Backright;
    private Servo Intake;
    private Servo Drone;
    private Servo Servoarm;
    private DcMotor arm;

    private Limelight3A limelight;
    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;

    private int armhighpos = 450;
    private int armlowpos = 30;
    private int ladderhighpos = 1900;
    private int ladderlowpos = 0;
	private int laddermidpos = 850;
    private int tgthPosition = 8400;
    private int tgthlPosition = 40;

    private int flencoderpos = 0;
    private int frencoderpos = 0;
    private int blencoderpos = 0;
    private int brencoderpos = 0;

    ElapsedTime timer = new ElapsedTime();

    public void init_LL() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        /*Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.*/
        limelight.start();
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

    public void init_navx() throws InterruptedException {
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope) navxMicro;
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        // Wait until the gyro calibration is complete
        timer.reset();
        while (navxMicro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            //noinspection BusyWait
            Thread.sleep(50);
        }
        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated.");
        telemetry.clear();
        telemetry.update();
    }
    public float run_navx() {

        AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
       /* telemetry.addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.update();*/
        return angles.firstAngle;
    }

    public void init_motors() {
        Intake = hardwareMap.get(Servo.class, "Intake");
        Servoarm = hardwareMap.get(Servo.class, "Servoarm");
        Drone = hardwareMap.get(Servo.class, "Drone");
        arm = hardwareMap.get(DcMotor.class, "arm");
        Backleft = hardwareMap.get(DcMotor.class, "Backleft");
        Backleft.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionBL", Backleft.getDirection());
        Backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeBL:", Backleft.getCurrentPosition());
        Backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Backright = hardwareMap.get(DcMotor.class, "Backright");
        Backright.setDirection(DcMotorSimple.Direction.FORWARD);
        Backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeBL:", Backright.getCurrentPosition());
        telemetry.addData("DcMotor DirectionBR", Backright.getDirection());
        Backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Frontleft = hardwareMap.get(DcMotor.class, "Frontleft");
        Frontleft.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionFL", Frontleft.getDirection());
        Frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeFL:", Frontleft.getCurrentPosition());
        Frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Frontright = hardwareMap.get(DcMotor.class, "Frontright");
        Frontright.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionFR", Frontright.getDirection());
        Frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeFR:", Frontright.getCurrentPosition());
        Frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Hook = hardwareMap.get(DcMotor.class, "Hook");
        Hook.setDirection(DcMotorSimple.Direction.REVERSE);
        Hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Hook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LadderLift = hardwareMap.get(DcMotor.class, "LadderLift");
        LadderLift.setDirection(DcMotorSimple.Direction.REVERSE);
        LadderLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LadderLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(0.0);
        telemetry.update();
    }

    public void Mecanumdriveyaw(double yawDegrees) {
        Frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        Backright.setDirection(DcMotorSimple.Direction.REVERSE);

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
            powerBR /= maxPower;  }
        Frontleft.setPower(powerFL);
        Frontright.setPower(powerFR);
        Backleft.setPower(powerBL);
        Backright.setPower(powerBR);
    }
    public void Mecanumdrive() {
        Frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        Backright.setDirection(DcMotorSimple.Direction.REVERSE);
        double h = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
        double robotAngle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI / 4;
        double rightX = gamepad1.left_stick_x;
        final double v1 = h * Math.sin(robotAngle) - rightX;
        final double v2 = h * Math.cos(robotAngle) + rightX;
        final double v3 = h * Math.cos(robotAngle) - rightX;
        final double v4 = h * Math.sin(robotAngle) + rightX;

        Frontleft.setPower(v1);
        Frontright.setPower(v2);
        Backleft.setPower(v3);
        Backright.setPower(v4);
    }

    public void Hook_Zero_Pwr_Behavior() {
        Hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void ladder_run_to_position0() {
        int tgt0Position = ladderlowpos;
        //LadderLift.setDirection(DcMotorSimple.Direction.FORWARD);
        LadderLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LadderLift.setTargetPosition(tgt0Position);
        LadderLift.setPower(1);
        LadderLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (LadderLift.isBusy()) {
            telemetry.addData("LadderPos:", LadderLift.getCurrentPosition());
            telemetry.update();
        }
    }
    public void ladder_run_to_position1() {
        int tgt1Position = laddermidpos;
//        LadderLift.setDirection(DcMotorSimple.Direction.FORWARD);
        LadderLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LadderLift.setTargetPosition(tgt1Position);
        LadderLift.setPower(1);
        LadderLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (LadderLift.isBusy()) {
            telemetry.addData("LadderPos:", LadderLift.getCurrentPosition());
            telemetry.update();
        }
    }
    public void ladder_run_to_position3() {
        int tgt3Position = ladderhighpos;
        LadderLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LadderLift.setTargetPosition(tgt3Position);
        LadderLift.setPower(1);
        LadderLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (LadderLift.isBusy()) {
            telemetry.addData("LadderPos:", LadderLift.getCurrentPosition());
            telemetry.update();
        }
    }
    public void hook_run_to_high_position() {
        Hook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Hook.setTargetPosition(tgthPosition);
        Hook.setPower(0.6);
        Hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Hook.isBusy()) {
            telemetry.addData("Hook Pos:", Hook.getCurrentPosition());
            telemetry.update();
        }
    }
    public void hook_run_to_zero_position() {
        Hook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Hook.setTargetPosition(tgthlPosition);
        Hook.setPower(0.6);
        Hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Hook.isBusy()) {
            telemetry.addData("Hook Pos:", Hook.getCurrentPosition());
            telemetry.update();
        }
        Hook_Zero_Pwr_Behavior();
    }


    @Override
    public void runOpMode() throws InterruptedException {

        init_motors();
        telemetry.addData("Status", "Actuators Initialized");
        telemetry.update();

        init_navx();
        init_LL();
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
                Intake.setPosition(0);
            } if (gamepad1.dpad_right) {
                Intake.setPosition(1);
            } if (gamepad1.left_bumper) {
                Servoarm.setPosition(0);
            } if (gamepad1.right_bumper) {
                Servoarm.setPosition(1);
            } if (gamepad1.dpad_up) {
                arm.setDirection(DcMotorSimple.Direction.FORWARD);
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setTargetPosition(armhighpos);
                arm.setPower(1.0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (arm.isBusy()) {
                    telemetry.addData("arm Pos:", arm.getCurrentPosition());
                    telemetry.update();
                }
            } if (gamepad1.dpad_down) {
                arm.setTargetPosition(armlowpos);
                arm.setPower(1.0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (arm.isBusy()) {
                    telemetry.addData("arm Pos:", arm.getCurrentPosition());
                    telemetry.update();
                }
            }

            /* LadderLift positions */ /* Drone Positions */
            if (gamepad2.a) {
                ladder_run_to_position0();
            } if (gamepad2.x) {
                ladder_run_to_position1();
            } if (gamepad2.y) {
                ladder_run_to_position3();
            }

            telemetry.update();
            idle();
        }
        limelight.stop();
    }

    String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}


