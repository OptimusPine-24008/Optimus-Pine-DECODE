/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


/*
 * This OpMode illustrates a teleop OpMode for an Omni robot.
 * An external "Robot" class is used to manage all motor/sensor interfaces, and to assist driving functions.
 * The IMU gyro is used to stabilize the heading when the operator is not requesting a turn.
 */

@TeleOp(name="Sample Teleop", group = "TeleOp")

public class SampleTeleop extends LinearOpMode {
    final double SAFE_DRIVE_SPEED = 0.8; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double SAFE_STRAFE_SPEED = 0.8; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double SAFE_YAW_SPEED = 0.5; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double HEADING_HOLD_TIME = 10.0; // How long to hold heading once all driver input stops. (This Avoids effects of Gyro Drift)

    private DcMotor armMotor = null; // it declairs the arm motor

    private Servo claw = null; // it declares the claw servo

    private CRServo intake = null; // it declares the



    // local parameters
    ElapsedTime stopTime   = new ElapsedTime();  // Use for timeouts.
    boolean autoHeading    = false; // used to indicate when heading should be locked.

    // get an instance of the "Robot" class.
    SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);

    @Override public void runOpMode()
    {
        // Initialize the drive hardware & Turn on telemetry
        robot.initialize(true);

        //initialize arm motor and servo
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        armMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        claw = hardwareMap.get(Servo.class, "claw");
        claw.scaleRange(0.2,0.8);

        intake = hardwareMap.crservo.get("intake");

        // Wait for driver to press start
        while(opModeInInit()) {
            telemetry.addData(">", "Touch Play to drive");

            // Read and display sensor data
            robot.readSensors();
            telemetry.update();
        }

        while (opModeIsActive()) {
            robot.readSensors();

            // Allow the driver to reset the gyro by pressing both small gamepad buttons
            if (gamepad1.options && gamepad1.share) {
                robot.resetHeading();
                robot.resetOdometry();
            }

            // read joystick values and scale according to limits set at top of this file
            double drive = -gamepad1.left_stick_y * SAFE_DRIVE_SPEED;      //  Fwd/back on left stick
            double strafe = -gamepad1.left_stick_x * SAFE_STRAFE_SPEED;     //  Left/Right on left stick
            double yaw = -gamepad1.right_stick_x * SAFE_YAW_SPEED;       //  Rotate on right stick


            //  OR... For special conditions, Use the DPAD to make pure orthoginal motions
            if (gamepad1.dpad_left) {
                strafe = SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_right) {
                strafe = -SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_up) {
                drive = SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_down) {
                drive = -SAFE_STRAFE_SPEED / 2.0;
            }

            // it will move the arm downwards if you press the right bumper
            //and upwards if you press the left bumper
            double armMotorPower;
            if (gamepad1.right_bumper) armMotorPower = 0.5;
            else if (gamepad1.left_bumper) armMotorPower = -0.5;
            else armMotorPower = 0;
            armMotor.setPower(armMotorPower);


            //press gamepad 1 a to close the claw, and b to open it
            double clawServoPower;
            if (gamepad1.cross) clawServoPower = 0.25;
            else clawServoPower = 0.55;
            if (gamepad1.circle) clawServoPower = 0.8;
            //the line below sets the current position of the servo,
            // expressed as a fraction of its available range.
            // If PWM power is enabled for the servo, the servo will attempt to move to the indicated position.
            claw.setPosition(clawServoPower);

            // press gamepad1 square to rotate clockwise and triangle to rotate counterclockwise
            if (gamepad1.square) {
                intake.setPower(1); // Forward at full speed
            } else if (gamepad1.triangle) {
                intake.setPower(-1); // Backward at full speed
            } else {
                intake.setPower(0); // Stop
            }


            // This is where we heep the robot heading locked so it doesn't turn while driving or strafing in a straight line.
            // Is the driver turning the robot, or should it hold its heading?
            if (Math.abs(yaw) > 0.05) {
                // driver is commanding robot to turn, so turn off auto heading.
                autoHeading = false;
            } else {
                // If we are not already locked, wait for robot to stop rotating (<2 deg per second) and then lock-in the current heading.
                if (!autoHeading && Math.abs(robot.getTurnRate()) < 2.0) {
                    robot.yawController.reset(robot.getHeading());  // Lock in the current heading
                    autoHeading = true;
                }
            }

            // If auto heading is on, override manual yaw with the value generated by the heading controller.
            if (autoHeading) {
                yaw = robot.yawController.getOutput(robot.getHeading());
            }

            //  Drive the wheels based on the desired axis motions
            robot.moveRobot(drive, strafe, yaw);

            // If the robot has just been sitting here for a while, make heading setpoint track any gyro drift to prevent rotating.
            if ((drive == 0) && (strafe == 0) && (yaw == 0)) {
                if (stopTime.time() > HEADING_HOLD_TIME) {
                    robot.yawController.reset(robot.getHeading());  // just keep tracking the current heading
                }
            } else {
                stopTime.reset();
            }
        }
    }

    public DcMotor getArmMotor() {
        return armMotor;
    }

    public void setArmMotor(DcMotor armMotor) {
        this.armMotor = armMotor;
    }
}