package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Game TeleOp Robot Centric", group = "StarterBot")
public class GameBotDecodeTeleOp extends OpMode {

    final double STOP_SPEED = 0.0;
    final double INTAKE_POWER = 1.0;
    final double SERVO_DEADBAND = 0.05;

    // Shooter velocities (ticks per second)
    private static final double SHOOTER_VEL_FAST = 4000.0;  // A button
    private static final double SHOOTER_VEL_SLOW = 1600.0;  // X button

    // Feeder servo limits
    private static final double FEEDER_MIN_POS = 0.00;
    private static final double FEEDER_MAX_POS = 0.18;

    // Motors
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotor intake;
    private DcMotorEx shooter;
    private DcMotor sideServo;
    private Servo leftFeeder, rightFeeder;

    private boolean shooterOn = false;
    private double shooterVelOut = 0.0;
    private double shooterVelMeas = 0.0;
    private boolean shooterSlowMode = false; // NEW toggle between fast/slow

    private enum IntakeState { STOPPED, INTAKING, OUTTAKING }
    private IntakeState intakeState;

    double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;
    double sideServoPower, leftFeederPos, rightFeederPos;

    @Override
    public void init() {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back");
        intake = hardwareMap.get(DcMotor.class, "intake");
        sideServo = hardwareMap.get(DcMotor.class, "perp");
        shooter = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder  = hardwareMap.get(Servo.class, "left_feeder");
        rightFeeder = hardwareMap.get(Servo.class, "right_feeder");

        // Directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        sideServo.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFeeder.setDirection(Servo.Direction.FORWARD);
        rightFeeder.setDirection(Servo.Direction.FORWARD);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeState = IntakeState.STOPPED;
        leftFeederPos = FEEDER_MIN_POS;
        rightFeederPos = FEEDER_MIN_POS;

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
        handleFeederServosGamepad1();
        handleIntakeControl();
        handleCRServoControl();
        handleShooterControl();

        telemetry.addData("Shooter", shooterOn ? (shooterSlowMode ? "ON (SLOW)" : "ON (FAST)") : "OFF");
        telemetry.addData("Shooter Cmd (t/s)", "%.0f", shooterVelOut);
        telemetry.addData("Shooter Meas (t/s)", "%.0f", shooterVelMeas);
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
        leftFrontPower  = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower   = (forward - strafe + rotate) / denominator;
        rightBackPower  = (forward + strafe - rotate) / denominator;
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    private void handleFeederServosGamepad1() {
        double lt = Range.clip(gamepad1.left_trigger,  FEEDER_MIN_POS, FEEDER_MAX_POS);
        double rt = Range.clip(gamepad1.right_trigger, FEEDER_MIN_POS, FEEDER_MAX_POS);
        leftFeederPos  = 1.00 - lt;
        rightFeederPos = rt;
        leftFeeder.setPosition(leftFeederPos);
        rightFeeder.setPosition(rightFeederPos);
    }

    private void handleIntakeControl() {
        if (gamepad2.left_bumper) intakeState = IntakeState.INTAKING;
        else if (gamepad2.right_bumper) intakeState = IntakeState.OUTTAKING;
        else intakeState = IntakeState.STOPPED;

        switch (intakeState) {
            case INTAKING: intake.setPower(INTAKE_POWER); break;
            case OUTTAKING: intake.setPower(-INTAKE_POWER); break;
            default: intake.setPower(STOP_SPEED);
        }
    }

    private void handleCRServoControl() {
        double rt = gamepad2.right_trigger;
        double lt = gamepad2.left_trigger;
        double power = rt - lt;
        if (Math.abs(power) < SERVO_DEADBAND) power = 0.0;
        sideServoPower = Range.clip(power, -0.5, 0.5);
        sideServo.setPower(sideServoPower);
    }

    private void handleShooterControl() {
        // Press A = fast, X = slow, B = stop
        if (gamepad2.a) {
            shooterOn = true;
            shooterSlowMode = false;
        }
        if (gamepad2.x) {
            shooterOn = true;
            shooterSlowMode = true;
        }
        if (gamepad2.b) shooterOn = false;

        if (shooterOn) {
            shooterVelOut = shooterSlowMode ? SHOOTER_VEL_SLOW : SHOOTER_VEL_FAST;
            shooter.setVelocity(shooterVelOut);
        } else {
            shooterVelOut = 0.0;
            shooter.setPower(0.0);
        }

        shooterVelMeas = shooter.getVelocity();
    }
}
