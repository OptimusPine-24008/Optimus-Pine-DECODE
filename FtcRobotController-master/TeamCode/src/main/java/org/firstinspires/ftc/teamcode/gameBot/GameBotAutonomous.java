package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.dfrobot.HuskyLens;

@Autonomous(name = "GameBotAutonomous", group = "StarterBot")
public class GameBotAutonomous extends LinearOpMode {

    // Positional servo positions
    private static final double FEED_POS = 0.18;
    private static final double REST_POS = 0.00;

    // Shooter timing
    private static final double FEED_TIME_SECONDS = 1.00;
    private static final double TIME_BETWEEN_SHOTS = 1.75;

    // Shooter velocity
    private static final double LAUNCHER_TARGET_VELOCITY = 1400;
    private static final double LAUNCHER_MIN_VELOCITY    = 1165;
    private static final PIDFCoefficients LAUNCHER_PIDF  = new PIDFCoefficients(300, 0, 0, 10);

    // Intake
    private static final double INTAKE_POWER = -1.00;

    // Drive timing
    private static final double DRIVE_HOLD_SEC = 0.75;
    private static final double TURN_HOLD_SEC  = 0.70;

    // HuskyLens tags
    private static final int ID_21_GPP = 1;
    private static final int ID_22_PGP = 3;
    private static final int ID_23_PPG = 2;
    private int detectedTag = -1;

    // Shots
    private int shotsToFire = 3;

    private enum Alliance { RED, BLUE }
    private Alliance alliance = Alliance.RED;

    private enum LaunchState { IDLE, PREPARE, LAUNCHING }
    private LaunchState launchState = LaunchState.IDLE;

    private enum AutoState {
        SELECT_ALLIANCE,
        INITIAL_DRIVE,
        SCAN_FOR_TAG,
        TAG_DECISION,

        TAG21_TURN,
        TAG21_PULSE_BOTH,
        TAG22_TURN,
        TAG22_PULSE_BOTH,
        TAG23_TURN,
        TAG23_PULSE_BOTH,

        TAG21_LAUNCH_SEQUENCE,
        TAG21_TURN_TO_FIELD,
        TAG21_TURN_TO_FIELD2,
        TAG21_DRIVE_OFF_LINE,
        TAG21_BACK_TO_LAUNCHING_ZONE,
        TAG21_WAIT_FOR_LAUNCH_CYCLE2,

        TAG22_LAUNCH_SEQUENCE,
        TAG22_WAIT_FOR_LAUNCH_CYCLE,
        TAG22_TURN_TO_FIELD,
        TAG22_TURN_TO_FIELD2,
        TAG22_DRIVE_OFF_LINE,
        TAG22_BACK_TO_LAUNCHING_ZONE,
        TAG22_WAIT_FOR_LAUNCH_CYCLE2,

        TAG23_LAUNCH_SEQUENCE,
        TAG23_WAIT_FOR_LAUNCH_CYCLE,
        TAG23_TURN_TO_FIELD,
        TAG23_TURN_TO_FIELD2,
        TAG23_DRIVE_OFF_LINE,
        TAG23_BACK_TO_LAUNCHING_ZONE,
        TAG23_WAIT_FOR_LAUNCH_CYCLE2,

        ESCAPE_THE_ZONE,
        COMPLETE
    }
    private AutoState autoState = AutoState.SELECT_ALLIANCE;

    // Hardware
    private DcMotorEx launcher;
    private Servo leftFeeder, rightFeeder;

    private DcMotor intake;
    private CRServo sorter;
    private HuskyLens huskyLens;

    private final ElapsedTime feederTimer = new ElapsedTime();
    private final ElapsedTime shotTimer   = new ElapsedTime();

    private SimplifiedOdometryRobot robot;

    @Override
    public void runOpMode() {

        robot = new SimplifiedOdometryRobot(this);
        robot.initialize(true);

        // Hardware mapping
        launcher    = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder  = hardwareMap.get(Servo.class, "left_feeder");
        rightFeeder = hardwareMap.get(Servo.class,"right_feeder");
        sorter      = hardwareMap.get(CRServo.class, "sorter");
        intake      = hardwareMap.get(DcMotor.class, "intake");

        // HuskyLens
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        if (!huskyLens.knock()) telemetry.addLine("HuskyLens NOT communicating!");
        else telemetry.addLine("HuskyLens OK");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        // Shooter
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(BRAKE);
        launcher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);

        // Positional servos
        leftFeeder.setDirection(Servo.Direction.REVERSE);
        rightFeeder.setDirection(Servo.Direction.FORWARD);
        stopFeeders();

        // Intake
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(BRAKE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        stopIntake();

        telemetry.addLine("INIT: X=Blue, B=Red, Triangle=Zero Heading");
        telemetry.update();

        // ===== INIT LOOP =====
        while (!isStarted() && !isStopRequested()) {

            if (gamepad1.b) alliance = Alliance.RED;
            if (gamepad1.x) alliance = Alliance.BLUE;
            if (gamepad1.y) robot.resetHeading();

            telemetry.addData("Alliance", alliance);
            telemetry.addData("Tag Seen", detectedTag == -1 ? "NONE" : detectedTag);
            telemetry.update();
            sleep(20);
        }

        if (isStopRequested()) return;

        robot.resetHeading();
        autoState   = AutoState.INITIAL_DRIVE;
        launchState = LaunchState.IDLE;
        shotTimer.reset();

        // ===== MAIN AUTO LOOP =====
        while (opModeIsActive()) {

            switch (autoState) {

                case INITIAL_DRIVE:
                    launcher.setVelocity(1470);
                    robot.drive(84.0, 0.6, 2.5);
                    sleep(300);
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

                case TAG21_TURN:
                    double tag21Goal = (alliance == Alliance.RED) ? 45.0 : -45.0;
                    robot.turnTo(tag21Goal, 0.6, TURN_HOLD_SEC);
                    sleep(200);
                    autoState = AutoState.TAG21_PULSE_BOTH;
                    break;

                case TAG21_PULSE_BOTH:
                    pulseServo(leftFeeder, FEED_POS, 500);
                    sorter.setPower(1.0);
                    double helpme = (alliance == Alliance.RED) ? 42.0 : -42.0;
                    robot.turnTo(helpme, 0.6, 0.65);
                    pulseServo(rightFeeder, FEED_POS, 500);
                    sleep(100);
                    double aaaah = (alliance == Alliance.RED) ? 45.0 : -45.0;
                    robot.turnTo(aaaah, 0.6, 0.65);
                    pulseServo(leftFeeder, FEED_POS, 500);
                    autoState = AutoState.TAG21_LAUNCH_SEQUENCE;
                    break;

                case TAG21_LAUNCH_SEQUENCE:
                    startIntake();
                    launcher.setVelocity(0);
                    double intake21 = (alliance == Alliance.RED) ? 0.0 : 0.0;
                    robot.turnTo(intake21, 0.6, TURN_HOLD_SEC);
                    robot.drive(-25.0, 0.6, 2.5);
                    sleep(200);
                    double iwannagotosleep = (alliance == Alliance.RED) ? 90.0 : -90.0;
                    robot.turnTo(iwannagotosleep, 0.6, 1.1);
                    autoState = AutoState.TAG21_TURN_TO_FIELD;
                    break;

                case TAG21_TURN_TO_FIELD:
                    robot.drive(11.02, 0.5, 2.0);
                    spinSorter(1.0, 1750);
                    sleep(150);
                    autoState = AutoState.TAG21_TURN_TO_FIELD2;
                    break;

                case TAG21_TURN_TO_FIELD2:
                    sorter.setPower(-1.0);
                    robot.drive(14.98, 0.10, 3.7);
                    sleep(100);
                    autoState = AutoState.TAG21_DRIVE_OFF_LINE;
                    break;

                case TAG21_DRIVE_OFF_LINE:
                    launcher.setVelocity(1450);
                    robot.drive(-18.5, 0.5, 2.2);
                    sorter.setPower(0.0);
                    autoState = AutoState.TAG21_BACK_TO_LAUNCHING_ZONE;
                    break;

                case TAG21_BACK_TO_LAUNCHING_ZONE:
                    double tspmo = (alliance == Alliance.RED) ? 0.0 : 0.0;
                    robot.turnTo(tspmo, 0.6, 1.1);
                    robot.drive(25.0, 0.5, 2.0);
                    double plswork = (alliance == Alliance.RED) ? 42.0 : -42.0;
                    robot.turnTo(plswork, 0.6, TURN_HOLD_SEC);
                    autoState = AutoState.TAG21_WAIT_FOR_LAUNCH_CYCLE2;
                    break;

                case TAG21_WAIT_FOR_LAUNCH_CYCLE2:
                    pulseServo(rightFeeder, FEED_POS, 500);
                    sleep(100);
                    sorter.setPower(-1.0);
                    double square = (alliance == Alliance.RED) ? 45.0 : -45.0;
                    robot.turnTo(square, 0.6, 0.65);
                    pulseServo(leftFeeder, FEED_POS, 500);
                    sleep(150);
                    double finalegoal21 = (alliance == Alliance.RED) ? 42.0 : -42.0;
                    robot.turnTo(finalegoal21, 0.6, 0.65);
                    pulseServo(rightFeeder, FEED_POS, 500);
                    sleep(100);
                    autoState = AutoState.ESCAPE_THE_ZONE;
                    break;

                case TAG22_TURN:
                    double tag22Goal = (alliance == Alliance.RED) ? 42.0 : -42.0;//32.5
                    robot.turnTo(tag22Goal, 0.6, TURN_HOLD_SEC);
                    autoState = AutoState.TAG22_PULSE_BOTH;
                    break;

                case TAG22_PULSE_BOTH:
                    pulseServo(rightFeeder, FEED_POS, 500);
                    spinSorter(-1.0, 1700);
                    double norway = (alliance == Alliance.RED) ? 45.0 : -45.0;
                    robot.turnTo(norway, 0.6, TURN_HOLD_SEC);
                    pulseServo(leftFeeder, FEED_POS, 500);
                    sleep(500);
                    double sweden = (alliance == Alliance.RED) ? 42.0 : -42.0;
                    robot.turnTo(sweden, 0.6, TURN_HOLD_SEC);
                    pulseServo(rightFeeder, FEED_POS, 500);
                    sleep(200);
                    autoState = AutoState.TAG22_LAUNCH_SEQUENCE;
                    break;

                case TAG22_LAUNCH_SEQUENCE:
                    startIntake();
                    launcher.setVelocity(0);
                    sorter.setPower(1.0);
                    autoState = AutoState.TAG22_WAIT_FOR_LAUNCH_CYCLE;
                    break;

                case TAG22_WAIT_FOR_LAUNCH_CYCLE:
                    double intake1 = (alliance == Alliance.RED) ? 90.0 : -90.0;
                    robot.turnTo(intake1, 0.6, TURN_HOLD_SEC);
                    autoState = AutoState.TAG22_TURN_TO_FIELD;
                    break;

                case TAG22_TURN_TO_FIELD:
                    robot.drive(11.0, 0.2, 2);
                    spinSorter(1.0, 2000);
                    sleep(200);
                    autoState = AutoState.TAG22_TURN_TO_FIELD2;
                    break;

                case TAG22_TURN_TO_FIELD2:
                    sorter.setPower(-1.0);
                    robot.drive(14.0, 0.09, 4.15);
                    sleep(200);
                    autoState = AutoState.TAG22_DRIVE_OFF_LINE;
                    break;

                case TAG22_DRIVE_OFF_LINE:
                    launcher.setVelocity(1450);
                    robot.drive(-20.0, 0.5, 2);
                    sorter.setPower(0);
                    autoState = AutoState.TAG22_BACK_TO_LAUNCHING_ZONE;
                    break;

                case TAG22_BACK_TO_LAUNCHING_ZONE:
                    double finalgoal = (alliance == Alliance.RED) ? 45.0 : -45.0;
                    robot.turnTo(finalgoal, 0.6, TURN_HOLD_SEC);
                    autoState = AutoState.TAG22_WAIT_FOR_LAUNCH_CYCLE2;
                    break;

                case TAG22_WAIT_FOR_LAUNCH_CYCLE2:
                    pulseServo(leftFeeder, FEED_POS, 500);
                    sleep(700);
                    spinSorter(1.0, 2000);
                    pulseServo(leftFeeder, FEED_POS, 500);
                    sleep(700);
                    double finalegoal = (alliance == Alliance.RED) ? 42.0 : -42.0;
                    robot.turnTo(finalegoal, 0.6, TURN_HOLD_SEC);
                    pulseServo(rightFeeder, FEED_POS, 500);
                    sleep(200);
                    autoState = AutoState.ESCAPE_THE_ZONE;
                    break;

                case TAG23_TURN:
                    double tag23Goal = (alliance == Alliance.RED) ? 42.0 : -42.0;
                    robot.turnTo(tag23Goal, 0.6, TURN_HOLD_SEC);
                    autoState = AutoState.TAG23_PULSE_BOTH;
                    break;

                case TAG23_PULSE_BOTH:
                    pulseServo(rightFeeder, FEED_POS, 500);
                    spinSorter(-1.0, 1700);
                    pulseServo(rightFeeder, FEED_POS, 500);
                    sleep(500);
                    double tag23Goal2 = (alliance == Alliance.RED) ? 45.0 : -45.0;
                    robot.turnTo(tag23Goal2, 0.6, TURN_HOLD_SEC);
                    pulseServo(leftFeeder, FEED_POS, 500);
                    sleep(200);
                    autoState = AutoState.TAG23_LAUNCH_SEQUENCE;
                    break;

                case TAG23_LAUNCH_SEQUENCE:
                    startIntake();
                    launcher.setVelocity(0);
                    sorter.setPower(1.0);
                    autoState = AutoState.TAG23_WAIT_FOR_LAUNCH_CYCLE;
                    break;

                case TAG23_WAIT_FOR_LAUNCH_CYCLE:
                    double intake3 = (alliance == Alliance.RED) ? 90.0 : -90.0;
                    robot.turnTo(intake3, 0.6, TURN_HOLD_SEC);
                    autoState = AutoState.TAG23_TURN_TO_FIELD;
                    break;

                case TAG23_TURN_TO_FIELD:
                    robot.drive(11.0, 0.2, 2);
                    spinSorter(1.0, 2000);
                    sleep(200);
                    autoState = AutoState.TAG23_TURN_TO_FIELD2;
                    break;

                case TAG23_TURN_TO_FIELD2:
                    sorter.setPower(-1.0);
                    robot.drive(14.0, 0.09, 4.15);
                    sleep(200);
                    autoState = AutoState.TAG23_DRIVE_OFF_LINE;
                    break;

                case TAG23_DRIVE_OFF_LINE:
                    launcher.setVelocity(1450);
                    robot.drive(-20.0, 0.5, 2);
                    spinSorter(1.0, 2000);
                    autoState = AutoState.TAG23_BACK_TO_LAUNCHING_ZONE;
                    break;

                case TAG23_BACK_TO_LAUNCHING_ZONE:
                    double finalgoal3 = (alliance == Alliance.RED) ? 45.0 : -45.0;
                    robot.turnTo(finalgoal3, 0.6, TURN_HOLD_SEC);
                    autoState = AutoState.TAG23_WAIT_FOR_LAUNCH_CYCLE2;
                    break;

                case TAG23_WAIT_FOR_LAUNCH_CYCLE2:
                    pulseServo(leftFeeder, FEED_POS, 500);
                    sleep(700);
                    spinSorter(1.0, 2000);
                    double finalgoal4 = (alliance == Alliance.RED) ? 42.0 : -42.0;
                    robot.turnTo(finalgoal4, 0.6, 0.3);
                    pulseServo(rightFeeder, FEED_POS, 500);
                    sleep(700);
                    double finalgoal5 = (alliance == Alliance.RED) ? 45.0 : -45.0;
                    robot.turnTo(finalgoal5, 0.6, 0.3);
                    pulseServo(leftFeeder, FEED_POS, 500);
                    sleep(200);
                    autoState = AutoState.ESCAPE_THE_ZONE;
                    break;

                case ESCAPE_THE_ZONE:
                    double escape = (alliance == Alliance.RED) ? 0.0 : 0.0;
                    robot.turnTo(escape, 0.6, TURN_HOLD_SEC);
                    robot.drive(-50.0, 0.75, 2);
                    autoState = AutoState.COMPLETE;
                    break;

                case COMPLETE:
                    robot.stopRobot();
                    stopFeeders();
                    stopIntake();
                    launcher.setVelocity(0);
                    sorter.setPower(0);
                    telemetry.addLine("AUTO COMPLETE");
                    telemetry.update();
                    sleep(50);
                    break;
            }

            telemetry.addData("AutoState", autoState);
            telemetry.addData("Tag", detectedTag);
            telemetry.update();
        }
    }

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
            case ID_21_GPP:
                autoState = AutoState.TAG21_TURN;
                break;

            case ID_22_PGP:
                autoState = AutoState.TAG22_TURN;
                break;

            case ID_23_PPG:
                autoState = AutoState.TAG23_TURN;
                break;

            default:
                autoState = AutoState.TAG21_TURN;
                break;
        }
    }

    private void requestLaunch() {
        launchState = LaunchState.PREPARE;
        shotTimer.reset();
        feederTimer.reset();
    }

    private boolean serviceLaunch(boolean shotRequested) {
        if (shotRequested) requestLaunch();

        switch (launchState) {

            case IDLE:
                break;

            case PREPARE:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    startFeeders();
                    feederTimer.reset();
                    launchState = LaunchState.LAUNCHING;
                }
                break;

            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    stopFeeders();
                    if (shotTimer.seconds() > TIME_BETWEEN_SHOTS) {
                        launchState = LaunchState.IDLE;
                        return true;
                    }
                }
                break;
        }
        return false;
    }

    private void startFeeders() {
        leftFeeder.setPosition(FEED_POS);
        rightFeeder.setPosition(FEED_POS);
    }

    private void stopFeeders() {
        leftFeeder.setPosition(REST_POS);
        rightFeeder.setPosition(REST_POS);
    }

    private void startIntake() {
        intake.setPower(INTAKE_POWER);
    }

    private void pulseServo(Servo servo, double toPos, long durationMs) {
        servo.setPosition(toPos);
        sleep(durationMs);
        servo.setPosition(REST_POS);
        sleep(100);
    }

    private void stopIntake() {
        intake.setPower(0.0);
    }

    private void spinSorter(double power, long durationMs) {
        sorter.setPower(power);
        sleep(durationMs);
        sorter.setPower(0.0);
    }

}
