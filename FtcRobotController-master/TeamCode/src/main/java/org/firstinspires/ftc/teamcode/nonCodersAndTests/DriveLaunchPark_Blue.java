package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;

/** DriveLaunchPark_Blue
 Minimal autonomous in the same style/format as your SampleAutonomous.java.
 Flow (robot starts ~50" facing the OBELISK):
 1) Read HuskyLens AprilTag (IDs 21/22/23)
 - 21 → GPP (launch order 2,1,3)
 - 22 → PGP (launch order 1,2,3)
 - 23 → PPG (launch order 1,3,2)
 2) Fire the three launchers IN PLACE (Lau_1P, Lau_2G, Lau_3P)
 3) Drive forward 60"
 4) Strafe 25" (positive = right; flip sign if needed)

 Exact command style:
 robot.drive(distanceInches, speed, kP);
 robot.turnTo(headingDeg, speed, kP);
 robot.strafe(distanceInches, speed, kP);
 */
@Autonomous(name = "DriveLaunchPark_Blue", group = "AutoBlue")
public class DriveLaunchPark_Blue extends LinearOpMode {

    // Vision IDs → patterns
    private static final int ID_21_GPP = 21; // fire 2,1,3
    private static final int ID_22_PGP = 22; // fire 1,2,3
    private static final int ID_23_PPG = 23; // fire 1,3,2

    // Simple stability/timeout for the read (no algorithm changes here)
    private static final int REQUIRED_STABLE_FRAMES = 3;
    private static final long VISION_TIMEOUT_MS     = 2500;

    // Launcher hardware names (set these in the RC config)
    private static final String HW_LAUNCH_1P = "Lau_1P"; // Purple
    private static final String HW_LAUNCH_2G = "Lau_2G"; // Green
    private static final String HW_LAUNCH_3P = "Lau_3P"; // Purple

    // Very simple firing timing (tune on-robot or swap for a feeder)
    private static final double LAUNCH_POWER = 1.0;
    private static final long   SPINUP_MS    = 400;
    private static final long   FIRE_MS      = 250;
    private static final long   COOLDOWN_MS  = 150;

    // Robot & hardware
    private final SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
    private HuskyLens huskyLens;
    private DcMotor launch1P, launch2G, launch3P;

    @Override public void runOpMode() throws InterruptedException {
        // Initialize robot hardware & turn on telemetry
        robot.initialize(true);

        // HuskyLens + Launchers
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        launch1P  = hardwareMap.get(DcMotor.class, HW_LAUNCH_1P);
        launch2G  = hardwareMap.get(DcMotor.class, HW_LAUNCH_2G);
        launch3P  = hardwareMap.get(DcMotor.class, HW_LAUNCH_3P);

        telemetry.addData(">", "Touch Play: DriveLaunchPark_Blue (Shoot→Drive 60 → Strafe 25)");
        telemetry.update();

        waitForStart();
        robot.resetHeading(); // Reset heading to set a baseline for Auto

        if (opModeIsActive()) {
            // ================= Read tag & determine order =================
            int tagId = detectStableTagId();
            int[] order = orderForTag(tagId);
            telemetry.addData("TagId", tagId);
            telemetry.addData("Pattern", patternForId(tagId));
            telemetry.addData("Firing order", Arrays.toString(order));
            telemetry.update();

            // ==================== Fire in place ====================
            fireSequence(order);

            // ======= Navigation in your exact call style (no extra vars) =======
            robot.drive( 60, 0.60, 0.15);  // forward 60 in
            robot.strafe(25, 0.60, 0.15);  // +right 25 in (flip sign if needed)
        }
    }

    // ---------------- Helpers ----------------
    private int detectStableTagId() {
        long end = System.currentTimeMillis() + VISION_TIMEOUT_MS;
        int stableId = -1;
        int sameCount = 0;

        while (opModeIsActive() && System.currentTimeMillis() < end) {
            HuskyLens.Block[] blocks = huskyLens.blocks();
            int chosenId = -1;
            int bestArea = -1;

            if (blocks != null) {
                for (HuskyLens.Block b : blocks) {
                    if (!isTargetId(b.id)) continue;
                    int area = Math.max(0, b.width) * Math.max(0, b.height);
                    if (area > bestArea) { bestArea = area; chosenId = b.id; }
                }
            }

            if (chosenId == -1) {
                sameCount = 0;
                stableId = -1;
            } else {
                if (chosenId == stableId) {
                    sameCount = Math.min(sameCount + 1, REQUIRED_STABLE_FRAMES);
                } else {
                    stableId = chosenId;
                    sameCount = 1;
                }
                if (sameCount >= REQUIRED_STABLE_FRAMES) {
                    return stableId;
                }
            }
        }
        // Fallback if timeout
        return ID_21_GPP; // conservative default (GPP → 2,1,3)
    }

    private boolean isTargetId(int id) {
        return id == ID_21_GPP || id == ID_22_PGP || id == ID_23_PPG;
    }

    private String patternForId(int id) {
        switch (id) {
            case ID_21_GPP: return "GPP";
            case ID_22_PGP: return "PGP";
            case ID_23_PPG: return "PPG";
            default: return "?";
        }
    }

    private int[] orderForTag(int id) {
        switch (id) {
            case ID_21_GPP: return new int[]{2,1,3}; // G, P, P
            case ID_22_PGP: return new int[]{1,2,3}; // P, G, P
            case ID_23_PPG: return new int[]{1,3,2}; // P, P, G
            default:        return new int[]{2,1,3}; // fallback to GPP
        }
    }

    private void fireSequence(int[] order) throws InterruptedException {
        for (int idx : order) {
            DcMotor m = (idx==1)? launch1P : (idx==2)? launch2G : launch3P;
            m.setPower(LAUNCH_POWER);
            sleep(SPINUP_MS);
            // If you have a feeder/trigger servo, actuate here instead of time windows
            sleep(FIRE_MS);
            m.setPower(0);
            sleep(COOLDOWN_MS);
        }
    }
}