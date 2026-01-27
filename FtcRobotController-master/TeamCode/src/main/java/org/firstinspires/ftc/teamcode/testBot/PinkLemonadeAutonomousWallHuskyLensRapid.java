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
@Autonomous(name = "PinkLemonadeAutonomusWallHuskyLensRapid", group = "StarterBot")
public class PinkLemonadeAutonomousWallHuskyLensRapid extends LinearOpMode {

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

    // HuskyLens tags
    private static final int ID_21_GPP = 1;
    private static final int ID_22_PGP = 2;
    private static final int ID_23_PPG = 3;
    private int detectedTag = -1;

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
        SCAN_FOR_TAG,
        TAG_DECISION,
        TAG21_SPINNY_SPIN,
        TAG22_SPINNY_SPIN,
        TAG23_SPINNY_SPIN,
        TAG21_LAUNCH_SEQUENCE,
        TAG22_LAUNCH_SEQUENCE,
        TAG23_LAUNCH_SEQUENCE,
        TAG21_WAIT_FOR_LAUNCH_CYCLE,
        TAG22_WAIT_FOR_LAUNCH_CYCLE,
        TAG23_WAIT_FOR_LAUNCH_CYCLE,
        TAG21_TURN_TO_FIELD,
        TAG22_TURN_TO_FIELD,
        TAG23_TURN_TO_FIELD,
        TAG22_DRIVE_BACK,
        TAG23_DRIVE_BACK,
        TAG21_NOM_NOM_1,
        TAG22_NOM_NOM_1,
        TAG23_NOM_NOM_1,
        TAG21_BACK_TO_LAUNCHING_ZONE,
        TAG22_BACK_TO_LAUNCHING_ZONE,
        TAG23_BACK_TO_LAUNCHING_ZONE,
        TAG21_WAIT_FOR_LAUNCH_CYCLE2,
        TAG22_WAIT_FOR_LAUNCH_CYCLE2,
        TAG23_WAIT_FOR_LAUNCH_CYCLE2,
        TAG21_ESCAPE_THE_ZONE,
        TAG22_ESCAPE_THE_ZONE,
        TAG23_ESCAPE_THE_ZONE,
        TAG21_IDK,
        TAG21_NOM_NOM_2,
        TAG22_NOM_NOM_2,
        TAG23_NOM_NOM_2,
        TAG21_NOM_NOM_3,
        TAG21_WAIT_FOR_LAUNCH_CYCLE3,
        COMPLETE
    }
    private AutoState autoState = AutoState.SELECT_ALLIANCE;

    // Hardware
    private DcMotorEx launcher;
    private CRServo leftFeeder, rightFeeder;
    private Servo gate;
    private DcMotor intake;
    private HuskyLens huskyLens;
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
        huskyLens   = hardwareMap.get(HuskyLens.class, "huskylens");

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

        // HuskyLens
        if (!huskyLens.knock()) telemetry.addLine("HuskyLens NOT communicating!");
        else telemetry.addLine("HuskyLens OK");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

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
            //telemetry.addData("Tag Recognized", );
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
                    autoState = AutoState.SCAN_FOR_TAG;
                    break;

                case SCAN_FOR_TAG:
                    detectedTag = -1;
                    readHuskyLensTag();
                    telemetry.addData("Tag After Drive", detectedTag);
                    telemetry.update();
                    autoState = AutoState.TAG_DECISION;
                    break;

                case TAG_DECISION:
                    runTagDecisionBranch();
                    break;

                case TAG21_SPINNY_SPIN:
                    robot.turnTo((alliance == Alliance.RED) ? -44.5 : 44.0, 0.95, 0.15);//initial value 0.25
                    autoState = AutoState.TAG21_LAUNCH_SEQUENCE;
                    break;
                case TAG22_SPINNY_SPIN:
                    robot.turnTo((alliance == Alliance.RED) ? -43.0 : 43.0, 0.95, 0.15);//initial value 0.25
                    autoState = AutoState.TAG22_LAUNCH_SEQUENCE;
                    break;
                case TAG23_SPINNY_SPIN:
                    robot.turnTo((alliance == Alliance.RED) ? -43.0 : 43.0, 0.95, 0.15);//initial valu 0.25
                    autoState = AutoState.TAG23_LAUNCH_SEQUENCE;
                    break;

                case TAG21_LAUNCH_SEQUENCE:
                    // stopIntake();
                    requestLaunch();
                    autoState = AutoState.TAG21_WAIT_FOR_LAUNCH_CYCLE;
                    break;
                case TAG22_LAUNCH_SEQUENCE:
                    // stopIntake();
                    requestLaunch();
                    autoState = AutoState.TAG22_WAIT_FOR_LAUNCH_CYCLE;
                    break;
                case TAG23_LAUNCH_SEQUENCE:
                    //stopIntake();
                    requestLaunch();
                    autoState = AutoState.TAG23_WAIT_FOR_LAUNCH_CYCLE;
                    break;

                // Wait for rapid fire shooting to finish
                case TAG21_WAIT_FOR_LAUNCH_CYCLE:
                    if (serviceLaunch(false)) {
                        //launcher.setVelocity(0);
                        stopFeeders();
                        gate.setPosition(GATE_CLOSE_POS);
                        autoState = AutoState.TAG21_TURN_TO_FIELD;
                    }
                    break;
                case TAG22_WAIT_FOR_LAUNCH_CYCLE:
                    if (serviceLaunch(false)) {
                        launcher.setVelocity(0);
                        stopFeeders();
                        gate.setPosition(GATE_CLOSE_POS);
                        autoState = AutoState.TAG22_TURN_TO_FIELD;
                    }
                    break;
                case TAG23_WAIT_FOR_LAUNCH_CYCLE:
                    if (serviceLaunch(false)) {
                        launcher.setVelocity(0);
                        stopFeeders();
                        gate.setPosition(GATE_CLOSE_POS);
                        autoState = AutoState.TAG23_TURN_TO_FIELD;
                    }
                    break;

                case TAG21_TURN_TO_FIELD:
                    robot.turnTo((alliance == Alliance.RED) ? 0.0 : 0.0, 0.95, 0.15);
                    startIntake();
                    autoState = AutoState.TAG21_NOM_NOM_1;
                    break;
                case TAG22_TURN_TO_FIELD:
                    robot.turnTo((alliance == Alliance.RED) ? 90.0 : -90.0, 0.95, 0.15);
                    startIntake();
                    autoState = AutoState.TAG22_DRIVE_BACK;
                    break;
                case TAG23_TURN_TO_FIELD:
                    robot.turnTo((alliance == Alliance.RED) ? 90.0 : -90.0, 0.95, 0.15);
                    startIntake();
                    autoState = AutoState.TAG23_DRIVE_BACK;
                    break;

                case TAG22_DRIVE_BACK:
                    robot.strafe(1, 0.95, 0.15);
                    autoState = AutoState.TAG22_NOM_NOM_1;
                    break;
                case TAG23_DRIVE_BACK:
                    robot.strafe(1, 0.95, 0.15);
                    autoState = AutoState.TAG23_NOM_NOM_1;
                    break;

                case TAG21_NOM_NOM_1:
                    leftFeeder.setPower(-1.0);
                    rightFeeder.setPower(-1.0);
                    robot.drive(59, 0.5, 0.25, 2.5);
                    stopFeeders();
                    //stopIntake();
                    autoState = AutoState.TAG21_BACK_TO_LAUNCHING_ZONE;
                    break;
                case TAG22_NOM_NOM_1:
                    leftFeeder.setPower(-1.0);
                    rightFeeder.setPower(-1.0);
                    robot.drive(59, 0.5, 0.25, 1.5);
                    stopFeeders();
                    //stopIntake();
                    autoState = AutoState.TAG22_BACK_TO_LAUNCHING_ZONE;
                    break;
                case TAG23_NOM_NOM_1:
                    leftFeeder.setPower(-1.0);
                    rightFeeder.setPower(-1.0);
                    robot.drive(59, 0.5, 0.25, 1.5);
                    stopFeeders();
                    //stopIntake();
                    autoState = AutoState.TAG23_BACK_TO_LAUNCHING_ZONE;
                    break;

                case TAG21_BACK_TO_LAUNCHING_ZONE:
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                    robot.drive(-57.0, 0.95, 0.25, 1.5);
                    robot.turnTo((alliance == Alliance.RED) ? -45.0 : 45.0, 0.95, 0.15); // changed
                    //shotsToFire = 4;
                    requestLaunch();
                    autoState = AutoState.TAG21_WAIT_FOR_LAUNCH_CYCLE2;
                    break;
                case TAG22_BACK_TO_LAUNCHING_ZONE:
                    robot.drive(-59.0, 0.95, 0.25, 1.5);
                    robot.turnTo((alliance == Alliance.RED) ? 43.0 : -43.0, 0.95, 0.15);
                    //shotsToFire = 4;
                    requestLaunch();
                    autoState = AutoState.TAG22_WAIT_FOR_LAUNCH_CYCLE2;
                    break;
                case TAG23_BACK_TO_LAUNCHING_ZONE:
                    robot.drive(-59.0, 0.95, 0.25, 1.5);
                    robot.turnTo((alliance == Alliance.RED) ? 43.0 : -43.0, 0.95, 0.15);
                    //shotsToFire = 4;
                    requestLaunch();
                    autoState = AutoState.TAG23_WAIT_FOR_LAUNCH_CYCLE2;
                    break;

                case TAG21_WAIT_FOR_LAUNCH_CYCLE2:
                    if (serviceLaunch(false)) {
                        startIntake();
                        // launcher.setVelocity(0);
                        stopFeeders();
                        // stopIntake();

                        gate.setPosition(GATE_CLOSE_POS);
                        autoState = AutoState.TAG21_ESCAPE_THE_ZONE;
                    }
                    break;
                case TAG22_WAIT_FOR_LAUNCH_CYCLE2:
                    if (serviceLaunch(false)) {
                        startIntake();
                        launcher.setVelocity(0);
                        stopFeeders();
                        // stopIntake();

                        gate.setPosition(GATE_CLOSE_POS);
                        autoState = AutoState.TAG22_ESCAPE_THE_ZONE;
                    }
                    break;
                case TAG23_WAIT_FOR_LAUNCH_CYCLE2:
                    if (serviceLaunch(false)) {
                        startIntake();
                        launcher.setVelocity(0);
                        stopFeeders();
                        // stopIntake();

                        gate.setPosition(GATE_CLOSE_POS);
                        autoState = AutoState.TAG23_ESCAPE_THE_ZONE;
                    }
                    break;

                case TAG21_ESCAPE_THE_ZONE:
                    robot.turnTo(0.0, 0.95, 0.25);
                    robot.strafe(35.0, 0.95, 0.15);
                    autoState = AutoState.TAG21_NOM_NOM_2;
                    break;
                case TAG22_ESCAPE_THE_ZONE:
                    robot.turnTo(0.0, 0.95, 0.15);
                    robot.strafe(35.0, 0.95, 0.15);
                    autoState = AutoState.TAG22_NOM_NOM_2;
                    break;
                case TAG23_ESCAPE_THE_ZONE:
                    robot.turnTo(0.0, 0.95, 0.15);
                    robot.strafe(35.0, 0.95, 0.15);
                    autoState = AutoState.TAG23_NOM_NOM_2;
                    break;

                case TAG21_NOM_NOM_2:
                    leftFeeder.setPower(-1.0);
                    rightFeeder.setPower(-1.0);
                    robot.drive(63, 0.55, 0.25, 1.5);
                    stopFeeders();
                    autoState = AutoState.TAG21_NOM_NOM_3;
                    break;
                case TAG22_NOM_NOM_2:
                    leftFeeder.setPower(-1.0);
                    rightFeeder.setPower(-1.0);
                    robot.drive(59, 0.5, 0.25, 1.5);
                    stopFeeders();
                    autoState = AutoState.COMPLETE;
                    break;
                case TAG23_NOM_NOM_2:
                    leftFeeder.setPower(-1.0);
                    rightFeeder.setPower(-1.0);
                    robot.drive(59, 0.5, 0.25, 1.5);
                    stopFeeders();
                    autoState = AutoState.COMPLETE;
                    break;

                case TAG21_NOM_NOM_3://return back to starfe to launch
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                    robot.drive(-60.0, 0.95, 0.25, 1.5);
                    robot.strafe(-65.0, 0.95, 0.15);
                    robot.turnTo((alliance == Alliance.RED) ? -45.0 : 45.0, 0.95, 0.15); // changed
                    //shotsToFire = 4;
                    requestLaunch();
                    autoState = AutoState.TAG21_WAIT_FOR_LAUNCH_CYCLE3;
                    break;

                case TAG21_WAIT_FOR_LAUNCH_CYCLE3:
                    if (serviceLaunch(false)) {
                        startIntake();
                        // launcher.setVelocity(0);
                        stopFeeders();
                        // stopIntake();

                        gate.setPosition(GATE_CLOSE_POS);
                        autoState = AutoState.TAG21_ESCAPE_THE_ZONE;
                    }
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
            telemetry.addData("Tag", detectedTag);
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

    // ======================== HUSKYLENS HELPERS ========================
    private void readHuskyLensTag() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        if (blocks == null) return;
        for (HuskyLens.Block b : blocks) {
            if (b.id == ID_21_GPP || b.id == ID_22_PGP || b.id == ID_23_PPG) {
                detectedTag = b.id;
                return;
            }
        }
    }

    private void runTagDecisionBranch() {
        switch (detectedTag) {
            case ID_21_GPP: autoState = AutoState.TAG21_SPINNY_SPIN; break;
            case ID_22_PGP: autoState = AutoState.TAG22_SPINNY_SPIN; break;
            case ID_23_PPG: autoState = AutoState.TAG23_SPINNY_SPIN; break;
            default:       autoState = AutoState.TAG21_SPINNY_SPIN; break;
        }
    }
}


