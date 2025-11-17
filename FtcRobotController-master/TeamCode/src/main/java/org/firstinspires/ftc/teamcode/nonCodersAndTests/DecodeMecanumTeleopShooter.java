package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Decode Sample TeleOp Starter + Mecanum Drive", group="Team 24008")
public class DecodeMecanumTeleopShooter extends LinearOpMode {

    // ===== Drive tuning (same as your SampleTeleop) =====
    final double SAFE_DRIVE_SPEED   = 0.8;
    final double SAFE_STRAFE_SPEED  = 0.8;
    final double SAFE_YAW_SPEED     = 0.5;
    final double HEADING_HOLD_TIME  = 10.0;

    // ===== Shooter tuning (minimal) =====
    final double LAUNCHER_TARGET_VELOCITY = 1125;   // ticks/sec
    final double LAUNCHER_MIN_VELOCITY    = 1075;   // must exceed before feeding
    final double FEED_TIME_SECONDS        = 0.20;   // how long to run feeders per shot
    final double FEEDER_STOP              = 0.0;
    final double FEEDER_FULL              = 1.0;
    final double TRIGGER_THRESHOLD        = 0.55;   // right trigger threshold

    // ===== Shooter hardware =====
    private DcMotorEx launcher;
    private CRServo leftFeeder, rightFeeder;

    // ===== Simple feed timing & edges =====
    private final ElapsedTime feedTimer = new ElapsedTime();
    private boolean feeding = false;
    private boolean flywheelOn = false;

    private boolean prevRightBumper = false;
    private boolean prevRightTriggerActive = false;

    // ===== Heading-hold helpers =====
    private final ElapsedTime stopTime = new ElapsedTime();
    private boolean autoHeading = false;

    // ===== Your existing mecanum + odom wrapper =====
    SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

    @Override
    public void runOpMode() {
        // Init drive/odom/imu
        robot.initialize(true);

        // Map shooter hardware
        launcher    = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder  = hardwareMap.get(CRServo.class,   "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class,   "right_feeder");

        // Launcher config: velocity mode + brake. (ABS checks below handle sign.)
        launcher.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);

        // Feeders: reverse one so both push the same direction
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(FEEDER_STOP);
        rightFeeder.setPower(FEEDER_STOP);

        while (opModeInInit()) {
            telemetry.addLine("> Touch PLAY to drive");
            robot.readSensors();
            telemetry.update();
        }

        while (opModeIsActive()) {
            // ===== Drive with heading hold (unchanged pattern) =====
            robot.readSensors();

            if (gamepad1.options && gamepad1.share) {
                robot.resetHeading();
                robot.resetOdometry();
            }

            double drive  = -gamepad1.left_stick_y * SAFE_DRIVE_SPEED;
            double strafe = -gamepad1.left_stick_x * SAFE_STRAFE_SPEED;
            double yaw    = -gamepad1.right_stick_x * SAFE_YAW_SPEED;

            if (gamepad1.dpad_left)  strafe =  SAFE_DRIVE_SPEED / 2.0;
            if (gamepad1.dpad_right) strafe = -SAFE_DRIVE_SPEED / 2.0;
            if (gamepad1.dpad_up)    drive  =  SAFE_DRIVE_SPEED / 2.0;
            if (gamepad1.dpad_down)  drive  = -SAFE_STRAFE_SPEED / 2.0;

            if (Math.abs(yaw) > 0.05) {
                autoHeading = false;
            } else if (!autoHeading && Math.abs(robot.getTurnRate()) < 2.0) {
                robot.yawController.reset(robot.getHeading());
                autoHeading = true;
            }
            if (autoHeading) yaw = robot.yawController.getOutput(robot.getHeading());

            robot.moveRobot(drive, strafe, yaw);

            if (drive == 0 && strafe == 0 && yaw == 0) {
                if (stopTime.time() > HEADING_HOLD_TIME) {
                    robot.yawController.reset(robot.getHeading());
                }
            } else {
                stopTime.reset();
            }

            // ===== Simple shooter controls =====
            // Toggle flywheel with Y
            if (gamepad1.y) {
                flywheelOn = true;
            }
            if (gamepad1.b) {
                flywheelOn = false;
            }
            launcher.setVelocity(flywheelOn ? LAUNCHER_TARGET_VELOCITY : 0);

            // Rising-edge detect: RB or RT>threshold fires one shot
            boolean rbNow = gamepad1.right_bumper;
            boolean rtNow = gamepad1.right_trigger > TRIGGER_THRESHOLD;
            boolean fireRequested = (rbNow && !prevRightBumper) || (rtNow && !prevRightTriggerActive);
            prevRightBumper = rbNow;
            prevRightTriggerActive = rtNow;

            double vel = launcher.getVelocity(); // could be negative; we’ll use abs
            boolean upToSpeed = Math.abs(vel) > LAUNCHER_MIN_VELOCITY;

            // If requested and up to speed, run feeders briefly
            if (fireRequested && upToSpeed && !feeding) {
                feeding = true;
                feedTimer.reset();
                leftFeeder.setPower(FEEDER_FULL);
                rightFeeder.setPower(FEEDER_FULL);
            }

            // Stop feeders after FEED_TIME_SECONDS
            if (feeding && feedTimer.seconds() > FEED_TIME_SECONDS) {
                feeding = false;
                leftFeeder.setPower(FEEDER_STOP);
                rightFeeder.setPower(FEEDER_STOP);
            }

            // ===== Teaching telemetry =====
            telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
            telemetry.addData("Launcher vel (tps)", "%.1f", vel);
            telemetry.addData("UpToSpeed(>%d)?", LAUNCHER_MIN_VELOCITY);
            telemetry.addData("Feeding", feeding);
            telemetry.update();
        }

        // Safe stop
        launcher.setVelocity(0);
        leftFeeder.setPower(FEEDER_STOP);
        rightFeeder.setPower(FEEDER_STOP);
        robot.stopRobot();
    }
}