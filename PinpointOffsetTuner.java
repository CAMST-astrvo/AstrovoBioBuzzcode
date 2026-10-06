package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

@TeleOp(name = "Pinpoint Offset Tuner", group = "Tuning")
public class PinpointOffsetTuner extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;

    /*
     * -----------------------------
     * OFFSET VALUES
     * -----------------------------
     */
    private double xOffset = 91.592;
    private double yOffset = 74.112;

    /*
     * -----------------------------
     * INCREMENTS
     * -----------------------------
     */
    private final double[] increments = {
            0.1,
            0.5,
            1.0,
            5.0,
            10.0
    };

    private int incrementIndex = 2;

    /*
     * -----------------------------
     * SAVE FILE
     * -----------------------------
     */
    private static final String FILE_NAME = "pinpoint_offsets.txt";

    /*
     * -----------------------------
     * BUTTON EDGE DETECTION
     * -----------------------------
     */

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    private boolean lastRB = false;
    private boolean lastLB = false;

    private boolean lastA = false;
    private boolean lastB = false;

    @Override
    public void runOpMode() throws InterruptedException {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        /*
         * -----------------------------
         * LOAD SAVED OFFSETS
         * -----------------------------
         */
        loadOffsets();

        /*
         * -----------------------------
         * APPLY OFFSETS
         * -----------------------------
         */
        pinpoint.setOffsets(xOffset, yOffset,DistanceUnit.MM);

        telemetry.addLine("================================");
        telemetry.addLine(" PINPOINT OFFSET TUNER READY");
        telemetry.addLine("================================");

        telemetry.addLine("");
        telemetry.addLine("DPAD UP/DOWN    -> Adjust X");
        telemetry.addLine("DPAD LEFT/RIGHT -> Adjust Y");

        telemetry.addLine("");
        telemetry.addLine("RB -> Larger Increment");
        telemetry.addLine("LB -> Smaller Increment");

        telemetry.addLine("");
        telemetry.addLine("A -> SAVE OFFSETS");
        telemetry.addLine("B -> RESET POSE");

        telemetry.addLine("");
        telemetry.addData("Loaded X", xOffset);
        telemetry.addData("Loaded Y", yOffset);

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            /*
             * -----------------------------
             * UPDATE PINPOINT
             * -----------------------------
             */
            pinpoint.update();

            /*
             * -----------------------------
             * INCREMENT CONTROL
             * -----------------------------
             */

            boolean rb = gamepad1.right_bumper;
            boolean lb = gamepad1.left_bumper;

            if (rb && !lastRB) {
                incrementIndex++;

                if (incrementIndex >= increments.length) {
                    incrementIndex = increments.length - 1;
                }
            }

            if (lb && !lastLB) {
                incrementIndex--;

                if (incrementIndex < 0) {
                    incrementIndex = 0;
                }
            }

            lastRB = rb;
            lastLB = lb;

            double increment = increments[incrementIndex];

            /*
             * -----------------------------
             * DPAD OFFSET CONTROL
             * -----------------------------
             */

            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadRight = gamepad1.dpad_right;

            /*
             * X OFFSET
             */

            if (dpadUp && !lastDpadUp) {
                xOffset += increment;
            }

            if (dpadDown && !lastDpadDown) {
                xOffset -= increment;
            }

            /*
             * Y OFFSET
             */

            if (dpadRight && !lastDpadRight) {
                yOffset += increment;
            }

            if (dpadLeft && !lastDpadLeft) {
                yOffset -= increment;
            }

            lastDpadUp = dpadUp;
            lastDpadDown = dpadDown;
            lastDpadLeft = dpadLeft;
            lastDpadRight = dpadRight;

            /*
             * -----------------------------
             * APPLY OFFSETS LIVE
             * -----------------------------
             */
            pinpoint.setOffsets(xOffset, yOffset,DistanceUnit.CM);

            /*
             * -----------------------------
             * SAVE OFFSETS
             * -----------------------------
             */

            boolean a = gamepad1.a;

            if (a && !lastA) {
                saveOffsets();
            }

            lastA = a;

            /*
             * -----------------------------
             * RESET POSE
             * -----------------------------
             */

            boolean b = gamepad1.b;

            if (b && !lastB) {
                pinpoint.resetPosAndIMU();
            }

            lastB = b;



            /*
             * -----------------------------
             * TELEMETRY
             * -----------------------------
             */

            telemetry.clearAll();

            telemetry.addLine("================================");
            telemetry.addLine(" PINPOINT OFFSET TUNER");
            telemetry.addLine("================================");

            telemetry.addLine("");

            telemetry.addData("X Offset", "%.3f", xOffset);
            telemetry.addData("Y Offset", "%.3f", yOffset);

            telemetry.addLine("");

            telemetry.addData("Increment", increment);

            telemetry.addLine("");

            /*
             * POSE TELEMETRY
             */

            telemetry.addLine("--------- POSE ---------");

            telemetry.addData(
                    "X Position",
                    "%.2f",
                    pinpoint.getPosition().getX(DistanceUnit.CM)
            );

            telemetry.addData(
                    "Y Position",
                    "%.2f",
                    pinpoint.getPosition().getY(DistanceUnit.CM)
            );

            telemetry.addData(
                    "Heading",
                    "%.2f",
                    Math.toDegrees(
                            pinpoint.getPosition().getHeading(AngleUnit.DEGREES)
                    )
            );

            telemetry.addLine("");

            /*
             * VELOCITY TELEMETRY
             */

            telemetry.addLine("------- VELOCITY -------");

            telemetry.addData(
                    "X Velocity",
                    "%.2f",
                    pinpoint.getVelX(DistanceUnit.CM)
            );

            telemetry.addData(
                    "Y Velocity",
                    "%.2f",
                    pinpoint.getVelY(DistanceUnit.CM)
            );

            telemetry.addData(
                    "Heading Velocity",
                    "%.2f",
                    Math.toDegrees(
                            pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)
                    )
            );

            telemetry.addLine("");

            /*
             * DEVICE STATUS
             */

            telemetry.addLine("-------- STATUS --------");

            telemetry.addData(
                    "Device Status",
                    pinpoint.getDeviceStatus()
            );

            telemetry.addData(
                    "Loop Frequency (Hz)",
                    "%.1f",
                    1000.0 / Math.max(1, getRuntime() * 1000)
            );

            telemetry.addLine("");

            /*
             * CONTROLS
             */

            telemetry.addLine("-------- CONTROLS --------");

            telemetry.addLine("DPAD UP/DOWN    -> X Offset");
            telemetry.addLine("DPAD LEFT/RIGHT -> Y Offset");

            telemetry.addLine("RB/LB           -> Increment");

            telemetry.addLine("A               -> SAVE");
            telemetry.addLine("B               -> RESET POSE");

            telemetry.addLine("");

            /*
             * TUNING GUIDE
             */

            telemetry.addLine("-------- TUNING --------");

            telemetry.addLine("Rotate robot 360 deg");
            telemetry.addLine("Observe X/Y drift");
            telemetry.addLine("Adjust offsets");
            telemetry.addLine("Repeat until drift minimized");

            telemetry.update();

            sleep(20);
        }
    }

    /*
     * =========================================================
     * SAVE OFFSETS
     * =========================================================
     */

    private void saveOffsets() {

        try {

            File file = new File(
                    AppUtil.getInstance().getSettingsFile(FILE_NAME).getAbsolutePath()
            );

            FileWriter writer = new FileWriter(file);

            writer.write(xOffset + "\n");
            writer.write(yOffset + "\n");

            writer.close();

            telemetry.addLine("OFFSETS SAVED");
            telemetry.update();

        } catch (IOException e) {

            telemetry.addLine("SAVE FAILED");
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
        }
    }

    /*
     * =========================================================
     * LOAD OFFSETS
     * =========================================================
     */

    private void loadOffsets() {

        try {

            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);

            if (!file.exists()) {
                return;
            }

            Scanner scanner = new Scanner(file);

            if (scanner.hasNextDouble()) {
                xOffset = scanner.nextDouble();
            }

            if (scanner.hasNextDouble()) {
                yOffset = scanner.nextDouble();
            }

            scanner.close();

        } catch (Exception e) {

            telemetry.addLine("LOAD FAILED");
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
        }
    }
}