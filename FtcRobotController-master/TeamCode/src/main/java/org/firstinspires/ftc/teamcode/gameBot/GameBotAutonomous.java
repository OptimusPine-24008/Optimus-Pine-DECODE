package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

/**
 * Simple HuskyLens AprilTag recognition
 *
 * Recognizes ONLY AprilTag IDs 21, 22, and 23 on the OBELISK.
 * The OBELISK is rotated so the tag faces the field; the tag is always centered
 * in front of the robot. We therefore do NOT refer to left/middle/right.
 *
 * ID → Pattern mapping:
 *  - 21 → GPP
 *  - 22 → PGP
 *  - 23 → PPG
 *
 * Run as a TeleOp to verify detection on the Driver Station. Pure vision only.
 */
@TeleOp(name = "Sensor: HuskyLens 21/22/23 (GPP/PGP/PPG)", group = "Sensor")
public class HuskyLensTagRecognition extends LinearOpMode {

    private static final String HUSKY_NAME = "huskylens";

    // Throttle telemetry so it’s readable
    private static final int READ_PERIOD_SECONDS = 1;

    private HuskyLens huskyLens;

    // Tag IDs with semantic names matching the OBELISK patterns
    private static final int ID_21_GPP = 1;
    private static final int ID_22_PGP = 3;
    private static final int ID_23_PPG = 2;

    @Override
    public void runOpMode() {
        huskyLens = hardwareMap.get(HuskyLens.class, HUSKY_NAME);

        Deadline rateLimit = new Deadline(READ_PERIOD_SECONDS, TimeUnit.SECONDS);
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with %s", huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "HuskyLens OK. Press START.");
        }

        // Ensure we’re in AprilTag mode
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) continue;
            rateLimit.reset();

            HuskyLens.Block[] blocks = huskyLens.blocks();
            int visible = (blocks == null ? 0 : blocks.length);
            telemetry.addData("Blocks visible", visible);

            // Choose the largest target (21/22/23) if any are present
            HuskyLens.Block best = null;
            int bestArea = -1;
            if (blocks != null) {
                for (HuskyLens.Block b : blocks) {
                    if (!isTargetId(b.id)) continue;
                    int area = Math.max(0, b.width) * Math.max(0, b.height);
                    if (area > bestArea) { bestArea = area; best = b; }
                }
            }

            if (best != null) {
                String pattern = patternForId(best.id);
                telemetry.addLine("Target tag detected");
                telemetry.addData("id", best.id);
                telemetry.addData("pattern", pattern);
                telemetry.addData("center (x,y)", best.x + ", " + best.y);
                telemetry.addData("size (w×h)", best.width + " × " + best.height);
                telemetry.addData("area", bestArea);
                telemetry.addData("note", "OBELISK tag centered; no L/M/R semantics");
            } else {
                telemetry.addLine("No target tag (21/22/23) visible");
            }

            telemetry.update();
        }
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
}
