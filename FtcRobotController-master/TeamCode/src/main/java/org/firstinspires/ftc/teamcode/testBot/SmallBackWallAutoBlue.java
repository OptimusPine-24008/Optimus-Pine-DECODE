package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.hardware.dfrobot.HuskyLens;
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
 * Fixed version with Rapid Fire
 */
@Autonomous(name = "SmallBackWallAutoBlue", group = "StarterBot")
public class SmallBackWallAutoBlue extends LinearOpMode {

    // Shooter/Feeder tuning
    private static final double FEED_TIME_SECONDS = 1.00;
    private static final double TIME_BETWEEN_SHOTS = 1.95;
    private static final double LAUNCHER_TARGET_VELOCITY = 1400;
    private static final double LAUNCHER_MIN_VELOCITY    = 1165;
    private static final PIDFCoefficients LAUNCHER_PIDF  = new PIDFCoefficients(200, 0.0015, 8, 13.5);
    // 300, 0, 0, 10
    // Intake tuning
    private static final double INTAKE_POWER = 1.00;

    // Drive tuning
    private static final double DRIVE_HOLD_SEC = 0.15;
    private static final double TURN_HOLD_SEC  = 0.15;

    // Gate positions
    final double GATE_CLOSE_POS = 0.77;
    final double GATE_OPEN_POS = 0.45;



    // Shots to fire at start
    //private int shotsToFire = 2;

    private enum Alliance { RED, BLUE }
    private Alliance alliance = Alliance.RED;

    // Rapid fire timing
    private static final double RAPID_FIRE_TIME   = 3.00; // seconds 3.55.- ...
    private static final double GATE_CLOSE_DELAY  = 0.75; // seconds
    private final ElapsedTime gateTimer = new ElapsedTime();

    private enum LaunchState { IDLE, SPIN_UP, RAPID_FIRE }
    private LaunchState launchState = LaunchState.IDLE;

    private enum AutoState {
        SELECT_ALLIANCE,
        INITIAL_DRIVE,
        SPINNY_SPIN,
        LAUNCH_SEQUENCE,
        WAIT_FOR_LAUNCH_CYCLE,
        PARK_AWAY,
        COMPLETE
    }
    private AutoState autoState = AutoState.SELECT_ALLIANCE;

    // Hardware
    private DcMotorEx launcher;
    private CRServo leftFeeder, rightFeeder;
    private Servo gate;
    private DcMotor intake;
    private SimplifiedOdometryRobot robot;

    // Helpers
    private final ElapsedTime feederTimer = new ElapsedTime();
    private final ElapsedTime shotTimer   = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize robot
        robot = new SimplifiedOdometryRobot(this);
        robot.initialize(true);

        // Map hardware
        launcher    = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder  = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        gate        = hardwareMap.get(Servo.class, "gate");
        gate.setDirection(Servo.Direction.REVERSE);
        intake      = hardwareMap.get(DcMotor.class, "intake");

        // Shooter config
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(BRAKE);
        launcher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);

        // Feeders
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        stopFeeders();

        // Intake
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(BRAKE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        stopIntake();


        gate.setPosition(GATE_CLOSE_POS);

        telemetry.addLine("Initialized. X=BLUE  B=RED  (during INIT)");
        telemetry.update();

        // INIT loop: alliance select + heading zero
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.b) alliance = Alliance.RED;
            if (gamepad1.x) alliance = Alliance.BLUE;
            if (gamepad1.y) robot.resetHeading();

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

        robot.resetHeading();
        autoState = AutoState.INITIAL_DRIVE;
        launchState = LaunchState.IDLE;
        shotTimer.reset();

        startIntake();

        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

        // Main autonomous loop
        while (opModeIsActive()) {
            switch (autoState) {


                case INITIAL_DRIVE:
                    startIntake();
                    robot.strafe(-110.0, 0.95, 0.25);//initial valu0.25
                    //robot.turnTo((alliance == Alliance.RED) ? -45.5 : 44.0, 0.95, 0.25);//initial value 0.25
                    autoState = AutoState.SPINNY_SPIN;
                    break;

                case SPINNY_SPIN:
                    robot.turnTo(-47.5, 0.95, 0.15);//initial value 0.25
                    autoState = AutoState.LAUNCH_SEQUENCE;
                    break;

                case LAUNCH_SEQUENCE:
                    // stopIntake();
                    requestLaunch();
                    autoState = AutoState.WAIT_FOR_LAUNCH_CYCLE;
                    break;

                case WAIT_FOR_LAUNCH_CYCLE:
                    if (serviceLaunch(false)) {
                        //launcher.setVelocity(0);
                        stopFeeders();
                        gate.setPosition(GATE_CLOSE_POS);
                        autoState = AutoState.PARK_AWAY;
                    }
                    break;

                case PARK_AWAY:
                    robot.turnTo(0.0, 0.95, 0.25);
                    robot.strafe(110.0, 0.95, 0.25);
                    robot.drive(30.0, 0.95, 0.25, 1.5);
                    autoState = AutoState.COMPLETE;
                    break;

                case COMPLETE:
                    robot.stopRobot();
                    stopFeeders();
                    telemetry.addLine("AUTO COMPLETE");
                    telemetry.addData("Heading (deg)", "%.1f", robot.getHeading());
                    telemetry.update();
                    sleep(50);
                    break;
            }

            // Telemetry
            telemetry.addData("AutoState", autoState);
            telemetry.addData("LaunchState", launchState);
            telemetry.addData("Heading (deg)", "%.1f", robot.getHeading());
            //telemetry.addData("Shots Remaining", shotsToFire);
            telemetry.addData("Launcher vel", "%.0f", launcher.getVelocity());
            telemetry.addData("Intake power", "%.2f", intake != null ? intake.getPower() : 0.0);
            telemetry.update();
            sleep(10);
        }
    }

    // ======================== LAUNCHER/FEEDER HELPERS ========================
    private void requestLaunch() {
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        feederTimer.reset();
        gate.setPosition(GATE_OPEN_POS);
        launchState = LaunchState.SPIN_UP;
    }

    private boolean serviceLaunch(boolean shotRequested) {
        switch (launchState) {

            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() >= LAUNCHER_MIN_VELOCITY) {
                    startFeeders();
                    feederTimer.reset();
                    launchState = LaunchState.RAPID_FIRE;
                }
                break;

            case RAPID_FIRE:
                // Feed continuously
                if (feederTimer.seconds() >= RAPID_FIRE_TIME) {
                    stopFeeders();
                    //gate.setPosition(GATE_CLOSE_POS);
                    gateTimer.reset();
                    launchState = LaunchState.IDLE;
                    //return true; // rapid fire complete
                }
                break;

            case IDLE:
                if (gateTimer.seconds() > GATE_CLOSE_DELAY) {
                    gate.setPosition(GATE_CLOSE_POS);   // safe close
                    return true;                        // launch fully complete
                }
                break;
            //default:
            //break;
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
        if (intake != null) intake.setPower(0.0); // set to 1.0 instead of 0.0 to keep spinning
    }
}
