package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TwoControllerTeleOp", group = "StarterBot")
public class TwoControllerTeleOp extends OpMode {

    final double STOP_SPEED = 0.0;
    final double INTAKE_POWER = 1.0;
    final double SERVO_DEADBAND = 0.05;

    private static final double SHOOTER_VEL_FAST = 1700.0;
    private static final double SHOOTER_VEL_SLOW = 1350.0;
    private static final double SHOOTER_VEL_REVERSE = -850.0;

    private static final double FEEDER_MIN_POS = 0.00;
    private static final double FEEDER_MAX_POS = 0.18;

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotor intake;
    private DcMotorEx shooter;
    private DcMotor sideServo;
    private Servo leftFeeder, rightFeeder;

    private boolean shooterOn = false;
    private double shooterVelOut = 0.0;
    private double shooterVelMeas = 0.0;
    private boolean shooterSlowMode = false;
    private boolean shooterReverseMode = false;

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
        sideServo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        sideServo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeState = IntakeState.STOPPED;

        leftFeederPos = FEEDER_MIN_POS;
        rightFeederPos = FEEDER_MIN_POS;

        sideServo.setPower(0.0);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

        // CONTROLLER 1 — DRIVE
        mecanumDrive(-gamepad1.left_stick_y, -gamepad1.right_stick_x, gamepad1.left_stick_x);

        // CONTROLLER 1 — FEEDERS
        handleFeederServosGamepad1();

        // CONTROLLER 2 — INTAKE
        handleIntakeControl();

        // CONTROLLER 2 — SORTER
        handleCRServoControl();

        // CONTROLLER 1 — SHOOTER
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

    // CONTROLLER 1 — FEEDERS (TRIGGERS)
    private void handleFeederServosGamepad1() {

        if (gamepad1.left_trigger > 0.05) {
            leftFeederPos  = 1.00 - FEEDER_MAX_POS;
            rightFeederPos = FEEDER_MIN_POS;
        }
        else if (gamepad1.right_trigger > 0.05) {
            leftFeederPos  = 1.00 - FEEDER_MIN_POS;
            rightFeederPos = FEEDER_MAX_POS;
        }
        else {
            leftFeederPos  = 1.00 - FEEDER_MIN_POS;
            rightFeederPos = FEEDER_MIN_POS;
        }

        leftFeeder.setPosition(leftFeederPos);
        rightFeeder.setPosition(rightFeederPos);
    }

    // CONTROLLER 2 — INTAKE (LB / RB)
    private void handleIntakeControl() {

        if (gamepad2.right_bumper) intakeState = IntakeState.INTAKING;
        else if (gamepad2.left_bumper) intakeState = IntakeState.OUTTAKING;
        else intakeState = IntakeState.STOPPED;

        switch (intakeState) {
            case INTAKING:
                intake.setPower(1.0);
                break;
            case OUTTAKING:
                intake.setPower(-1.0);
                break;
            default:
                intake.setPower(STOP_SPEED);
        }
    }

    // CONTROLLER 2 — SORTER (TRIGGERS)
    private void handleCRServoControl() {

        if (gamepad2.right_trigger > 0.05) {
            sideServoPower = 0.5;
        }
        else if (gamepad2.left_trigger > 0.05) {
            sideServoPower = -0.5;
        }
        else {
            sideServoPower = 0.0;
        }

        sideServo.setPower(sideServoPower);
    }

    // CONTROLLER 1 — SHOOTER
    private void handleShooterControl() {

        if (gamepad1.a) {
            shooterOn = true;
            shooterSlowMode = false;
            shooterReverseMode = false;
        }
        if (gamepad1.x) {
            shooterOn = true;
            shooterSlowMode = true;
            shooterReverseMode = false;
        }
        if (gamepad1.y) {
            shooterOn = true;
            shooterReverseMode = true;
        }
        if (gamepad1.b) {
            shooterOn = false;
            shooterReverseMode = false;
        }

        if (shooterOn) {
            if (shooterReverseMode) {
                shooterVelOut = SHOOTER_VEL_REVERSE;
            } else {
                shooterVelOut = shooterSlowMode ? SHOOTER_VEL_SLOW : SHOOTER_VEL_FAST;
            }
            shooter.setVelocity(shooterVelOut);
        } else {
            shooterVelOut = 0.0;
            shooter.setPower(0.0);
        }

        shooterVelMeas = shooter.getVelocity();
    }
}
