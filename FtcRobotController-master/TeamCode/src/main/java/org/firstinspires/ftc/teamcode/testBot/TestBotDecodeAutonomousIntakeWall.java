package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TestBotDecodeAutonomousIntake
 * This code assumes the following hardware:
 *  - Mecanum drive + IMU + odometry via SimplifiedOdometryRobot
 *  - Launcher + dual feeders with the same state machine style as StarterBotAuto
 * Hardware names MUST match StarterBotTeleopMecanums:
 *  left_front_drive, right_front_drive, left_back_drive, right_back_drive
 *  launcher, left_feeder, right_feeder
 * Additional devices expected by SimplifiedOdometryRobot:
 *  imu, axial (drive odom), lateral (strafe odom)
 * EXTRA:
 *  intake (goBILDA Yellow Jacket 5203 Series) — mapped here and used during auto
 */
@Autonomous(name = "TestBotDecodeAutonomousIntake Wall", group = "StarterBot")
public class TestBotDecodeAutonomousIntakeWall extends LinearOpMode {

    // Shooter/Feeder tuning
    private static final double FEED_TIME_SECONDS = 1.00; //0.4
    private static final double TIME_BETWEEN_SHOTS = 1.95; //3.0, 1.75

    private static final double LAUNCHER_TARGET_VELOCITY = 1400;  // ticks/s //1350 now 1360
    private static final double LAUNCHER_MIN_VELOCITY    = 1165;  // ticks/s //1125 now 1135
    private static final PIDFCoefficients LAUNCHER_PIDF  = new PIDFCoefficients(300, 0, 0, 10);

    // Intake tuning — simple open-loop
    private static final double INTAKE_POWER = 1.00; // flip sign or motor direction if needed

    // Drive tuning
    private static final double DRIVE_HOLD_SEC = 0.75;
    private static final double TURN_HOLD_SEC  = 1.0;

    //gate position
    final double GATE_CLOSE_POS = 0.80;
    final double GATE_OPEN_POS = 0.45;

    // How many shots to fire at start
    private int shotsToFire = 3;

    private enum Alliance { RED, BLUE }
    private Alliance alliance = Alliance.RED;

    private enum LaunchState { IDLE, PREPARE, LAUNCHING }
    private LaunchState launchState = LaunchState.IDLE;

    private enum AutoState {
        SELECT_ALLIANCE,
        INITIAL_DRIVE,
        LAUNCH_SEQUENCE,
        WAIT_FOR_LAUNCH_CYCLE,
        TURN_TO_FIELD,
        DRIVE_OFF_LINE,
        BACK_TO_LAUNCHING_ZONE,
        WAIT_FOR_LAUNCH_CYCLE2,
        ESCAPE_THE_ZONE,
        COMPLETE
    }
    private AutoState autoState = AutoState.SELECT_ALLIANCE;

    // Hardware
    private DcMotorEx launcher;
    private CRServo leftFeeder, rightFeeder;
    private Servo gate;
    private DcMotor intake; // <<< NEW

    // Helpers
    private final ElapsedTime feederTimer = new ElapsedTime();
    private final ElapsedTime shotTimer   = new ElapsedTime();

    // Drivetrain/odometry wrapper
    private SimplifiedOdometryRobot robot;

    @Override
    public void runOpMode() {
        // Build the odometry+IMU robot wrapper
        robot = new SimplifiedOdometryRobot(this);
        robot.initialize(true); // show internal telemetry

        // Map launcher + feeders
        launcher    = hardwareMap.get(DcMotorEx.class,   "launcher");
        leftFeeder  = hardwareMap.get(CRServo.class,     "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class,     "right_feeder");
        gate        = hardwareMap.get(Servo.class,       "gate");

        gate.setDirection(Servo.Direction.REVERSE);

        // Map intake (goBILDA 5203 Yellow Jacket)
        intake = hardwareMap.get(DcMotor.class, "intake"); // <<< NEW

        // Shooter configuration
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(BRAKE);
        launcher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);

        // Feeders: reverse left so both feed inward together
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        stopFeeders();

        // Intake basic config (open-loop)
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   // simple power control
        intake.setZeroPowerBehavior(BRAKE);
        intake.setDirection(DcMotor.Direction.FORWARD);        // flip if rollers spin wrong way
        stopIntake();

        telemetry.addLine("Initialized. X=BLUE  B=RED  (during INIT)");
        telemetry.update();

        gate.setPosition(GATE_CLOSE_POS);

        // INIT loop: alliance select + heading zero
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.b) alliance = Alliance.RED;
            if (gamepad1.x) alliance = Alliance.BLUE;

            if (gamepad1.y) {
                robot.resetHeading();
            }

            telemetry.addData("Press square", "for BLUE");
            telemetry.addData("Press circle", "for RED");
            telemetry.addData("Selected Alliance", alliance);
            telemetry.addData("Press triangle", "zero heading");
            telemetry.addData("Heading (deg)", "%.1f", robot.getHeading());
            telemetry.addLine("Aim robot at field starting position, press triangle to zero");
            telemetry.update();
            sleep(20);
        }

        if (isStopRequested()) return;

        // Reset heading for precise motion
        robot.resetHeading();

        autoState   = AutoState.INITIAL_DRIVE;
        launchState = LaunchState.IDLE;
        shotTimer.reset();

        // Main autonomous state machine
        while (opModeIsActive()) {
            switch (autoState) {
                case INITIAL_DRIVE:
                    // Begin intaking while we drive away
                    startIntake(); // <<< NEW
                    robot.drive(110.0, 0.8, DRIVE_HOLD_SEC);//-50.0
                    double targetHeading = (alliance == Alliance.RED) ? 43.0 : -43.0; //45
                    robot.turnTo(targetHeading, 0.75, 0.25);
                    autoState = AutoState.LAUNCH_SEQUENCE;
                    break;

                case LAUNCH_SEQUENCE:
                    // Pause intake while we shoot to avoid jams / stray feeds
                    stopIntake(); // <<< NEW
                    requestLaunch(); // sets state & timers
                    autoState = AutoState.WAIT_FOR_LAUNCH_CYCLE;
                    break;

                case WAIT_FOR_LAUNCH_CYCLE:
                    if (serviceLaunch(false)) {
                        shotsToFire--;
                        if (shotsToFire > 0) {
                            // Start next shot
                            requestLaunch();
                        }
                        else {
                            // Done firing
                            launcher.setVelocity(0);
                            stopFeeders();
                            gate.setPosition(GATE_CLOSE_POS);
                            autoState = AutoState.TURN_TO_FIELD;
                        }
                    }
                    break;

                case TURN_TO_FIELD:
                    // Feeder spin logic removed from here as requested
                    double targetHeading2 = (alliance == Alliance.RED) ? 90.0 : -90.0; //45
                    robot.turnTo(targetHeading2, 0.75, 0.25);
                    //robot.strafe(45.0, 0.75, 0.5);
                    startIntake();//this code will collect two artifacts
                    autoState = AutoState.DRIVE_OFF_LINE;
                    break;

                case DRIVE_OFF_LINE:
                    // START: Set both feeders to move backward (opposite direction from launching)
                    leftFeeder.setPower(-1.0);
                    rightFeeder.setPower(-1.0);

                    robot.drive(58.5, /*maxPower*/0.30, DRIVE_HOLD_SEC);//original -95.0*

                    // STOP: Stop the backward feeder motion after the drive completes
                    stopFeeders();

                    //the code below will stop intKE WHILE DRVING OFF LINE
                    stopIntake();

                    autoState = AutoState.BACK_TO_LAUNCHING_ZONE;
                    break;

                case BACK_TO_LAUNCHING_ZONE:
                    robot.drive(-59.0, /*maxPower*/0.95, DRIVE_HOLD_SEC);
                    //robot.strafe(-45.0, 0.75, 0.5);
                    double targetHeading3 = (alliance == Alliance.RED) ? 43.0 : -43.0;//135.0, -135.0
                    robot.turnTo(targetHeading3, /*maxPower*/0.75, 0.5);

                    //Below the intake will activate to assist launch
                    //startIntake();Move this line to after requestLaunch() to execute intake with launch

                    // ---Change request shot to two shots---
                    //This code will reset shotsToFire to 4
                    shotsToFire = 4;

                    //---Now it is ready to request shotToFire---
                    requestLaunch();
                    autoState = AutoState.WAIT_FOR_LAUNCH_CYCLE2;
                    break;

                case WAIT_FOR_LAUNCH_CYCLE2:
                    if (serviceLaunch(false)) {
                        shotsToFire--;
                        if (shotsToFire > 0) {
                            // Start next shot
                            requestLaunch();
                            startIntake();
                        }
                        else {
                            // Done firing
                            launcher.setVelocity(0);
                            stopFeeders();
                            stopIntake();
                            gate.setPosition(GATE_CLOSE_POS);
                            autoState = AutoState.ESCAPE_THE_ZONE;
                        }
                    }
                    break;

                case ESCAPE_THE_ZONE:
                    double targetHeading4 = (alliance == Alliance.RED) ? 0.0 : 0.0;
                    robot.turnTo(targetHeading4, /*maxPower*/0.6, 0.25);
                    robot.drive(-60.0, /*maxPower*/0.75, 0.5);
                    autoState = AutoState.COMPLETE;

                case COMPLETE:
                    robot.stopRobot();
                    stopFeeders();
                    //stopIntake(); // <<< NEW ---Maybe we don't need this line
                    telemetry.addLine("AUTO COMPLETE");
                    telemetry.addData("Heading (deg)", "%.1f", robot.getHeading());
                    telemetry.update();
                    sleep(50);
                    break;
            }

            // Helpful live telemetry each loop
            telemetry.addData("AutoState", autoState);
            telemetry.addData("LaunchState", launchState);
            telemetry.addData("Heading (deg)", "%.1f", robot.getHeading());
            telemetry.addData("Shots Remaining", shotsToFire);
            telemetry.addData("Launcher vel", "%.0f", launcher.getVelocity());
            telemetry.addData("Intake power", "%.2f", intake != null ? intake.getPower() : 0.0);
            telemetry.update();
            sleep(10);
        }
    }

    // ======================== LAUNCHER/FEEDER STATE MACHINE ========================

    /** Call once when you want to queue a shot */
    private void requestLaunch() {
        launchState = LaunchState.PREPARE;
        shotTimer.reset();
        feederTimer.reset();
        gate.setPosition(GATE_OPEN_POS);
    }

    /**
     * Drives the launcher/feeder sequence. Returns true for ONE cycle right after a ball is launched
     * and the inter-shot delay has elapsed.
     * @param shotRequested pass true if you need to (re)start the cycle, false to continue processing
     */
    private boolean serviceLaunch(boolean shotRequested) {
        if (shotRequested) requestLaunch();

        switch (launchState) {
            case IDLE:
                break;

            case PREPARE:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    // Feed for a short pulse
                    startFeeders();
                    feederTimer.reset();
                    launchState = LaunchState.LAUNCHING;
                }
                break;

            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    stopFeeders();
                    // Wait between shots to stabilize RPM/aim
                    if (shotTimer.seconds() > TIME_BETWEEN_SHOTS) {
                        launchState = LaunchState.IDLE;
                        return true; // signal "one shot completed"
                    }
                }
                break;
        }
        return false;
    }

    private void startFeeders() {
        leftFeeder.setPower(1.0);
        rightFeeder.setPower(1.0);
    }

    private void stopFeeders() {
        leftFeeder.setPower(0.0);
        rightFeeder.setPower(0.0);
    }

    // ======================== INTAKE HELPERS ========================
    private void startIntake() {
        if (intake != null) intake.setPower(INTAKE_POWER);
    }

    private void stopIntake() {
        if (intake != null) intake.setPower(0.0);
    }
}
