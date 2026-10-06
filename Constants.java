package org.firstinspires.ftc.teamcode.astrovos.util;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public final class Constants {

    private Constants() {
    }

    public static final class Hardware {

        public static final String LF_NAME = "FrontLeftMotor";
        public static final String RF_NAME = "FrontRightMotor";
        public static final String LB_NAME = "BackLeftMotor";
        public static final String RB_NAME = "BackRightMotor";
        public static final String PINPOINT_NAME = "pinpoint";
        public static final String LIMELIGHT_NAME = "limelight";
        public static final String TURRET_NAME = "turret";
        public static final String TURRET_HOME_SWITCH_NAME = "turretHome";
        public static final String HOOD_SERVO_NAME = "angle";
        public static final String SHOT_1_NAME = "shot1";
        public static final String SHOT_2_NAME = "shot2";
        public static final String INTAKE_NAME = "intake";
        public static final String STOPPER_NAME = "stopper";
        public static final String KICKER_NAME = "kick";

        private Hardware() {
        }
    }

    public static final class Localization {

        // Tune only if the pod offsets in the copied Quickstart project are wrong for this robot.
        public static final double X_POD_OFFSET_MM = 91.592;
        public static final double Y_POD_OFFSET_MM = 74.112;

        /*
         * Keep false if Pedro/Pinpoint already owns the field pose before this OpMode.
         * Set true only after tuning the start poses below for this robot's actual starting tiles.
         */
        public static final boolean RESET_PINPOINT_TO_START_POSE_ON_ALLIANCE_SELECT = true;
        public static final double BLUE_START_X_INCHES = 72.0;
        public static final double BLUE_START_Y_INCHES = 72.0;
        public static final double BLUE_START_HEADING_DEGREES = 90.0;
        public static final double RED_START_X_INCHES = 72.0;
        public static final double RED_START_Y_INCHES = 72.0;
        public static final double RED_START_HEADING_DEGREES = 90.0;
        public static final boolean USE_LAST_AUTO_POSE_IN_TELEOP = false;
        public static final double BLUE_LAST_AUTO_HEADING_OFFSET_DEGREES = 0.0;
        public static final double RED_LAST_AUTO_HEADING_OFFSET_DEGREES = 180.0;
        public static final double VELOCITY_FILTER_ALPHA = 0.45;

        private Localization() {
        }
    }

    public static final class Drive {

        public static final boolean DRIVE_ENABLED = true;
        public static final double DEADBAND = 0.04;
        public static final double NORMAL_SPEED = 1;
        public static final double PRECISION_SPEED = 0.62;
        public static final double STRAFE_MULTIPLIER = 1.7;

        private Drive() {
        }
    }

    public static final class Turret {

        /*
         * Tune first: replace this with the exact encoder ticks per output shaft revolution
         * for the installed goBILDA 5203 motor cartridge. The 31:114 value below is only
         * the external turret gear stage, not the motor gearbox encoder count.
         */
        public static final double MOTOR_TICKS_PER_OUTPUT_REV = 383.6;
        public static final double DRIVER_GEAR_TEETH = 31.0;
        public static final double DRIVEN_GEAR_TEETH = 114.0;
        public static final double TICKS_PER_TURRET_DEGREE =
                MOTOR_TICKS_PER_OUTPUT_REV
                        * (DRIVEN_GEAR_TEETH / DRIVER_GEAR_TEETH)
                        / 360.0;

        /*
         * Robot-relative turret angle convention:
         * 0 deg = robot front, +90 deg = left side, +180 deg = back, -90 deg = right side.
         * The touch-sensor home is 140 deg clockwise from front, so it is -140 deg here.
         * The opposite hard limit is 170 deg counter-clockwise from front, so it is +170 deg.
         * Encoder zero is captured whenever that physical switch is pressed.
         */
        public static final double HOME_ROBOT_RELATIVE_DEGREES = -120.0;
        public static final double MIN_ROBOT_RELATIVE_DEGREES = -120.0;
        public static final double MAX_ROBOT_RELATIVE_DEGREES = 125.0;
        public static final double LIMIT_GUARD_DEGREES = 4.0;

        public static final double HOME_SWITCH_DEBOUNCE_SECONDS = 0.04;
        public static final double HOME_SEARCH_ANGLE_DIRECTION = -1.0;
        public static final double HOME_SEARCH_POWER = 0.25;
        public static final double HOME_SEARCH_TIMEOUT_SECONDS = 8.0;
        public static final double HOME_STALL_CHECK_SECONDS = 0.50;
        public static final double HOME_STALL_MIN_MOVEMENT_DEGREES = 0.50;
        public static final double HOME_TARGET_TOLERANCE_DEGREES = 0.25;

        /*
         * Clockwise motion decreases encoder ticks and turret angle; counter-clockwise motion
         * increases both. With the motor configured FORWARD, positive power increases angle.
         */
        public static final double POWER_SIGN = 1.0;
        public static final double ENCODER_TICK_TO_DEGREES_SIGN = 1.0;
        public static final double KP = 0.025;
        public static final double KD = 0.0031;
        public static final double MAX_POWER = 0.7;
        public static final double HOLD_POWER_MAX = 0.1;
        public static final double READY_TOLERANCE_DEGREES = 1.25;
        public static final double READY_ERROR_RATE_DEGREES_PER_SECOND = 35.0;

        private Turret() {
        }
    }

    public static final class Hood {

        // Placeholder servo limits. Verify the real safe min/max before full-speed shots.
        public static final double HOOD_MIN_POSITION = 0.40;
        public static final double HOOD_MAX_POSITION = 0;
        public static final double HOOD_PARK_POSITION = 0.40;

        /*
         * First-pass distance map. Replace these with measured distance-to-servo-position data.
         * Every point must remain inside HOOD_MIN_POSITION and HOOD_MAX_POSITION.
         */
        public static final InterpolatingTable.Point[] HOOD_MAP = {
                new InterpolatingTable.Point( 15.6, 0.40),
              new InterpolatingTable.Point(88.6 , 0),
                new InterpolatingTable.Point(108.6,0),
                new InterpolatingTable.Point(144.1,0)



        };

        /*
         * If shots drop when voltage sags or balls pull speed down, tune these from real data.
         * Start at 0.0 if the correction fights the shot.
         */
        public static final double HOOD_POSITION_PER_LOW_VOLT = 0.010;
        public static final double HOOD_POSITION_PER_100_TPS_LOW = 0.006;
        public static final double SETTLE_TIME_SECONDS = 0.0;
        public static final double TARGET_CHANGE_EPSILON = 0.003;

        private Hood() {
        }
    }

    public static final class Shooter {

        /*
         * If either wheel spins the wrong way, change only these directions first.
         * This assumes shot1 and shot2 are opposing flywheels.
         */
        public static final DcMotorSimple.Direction SHOT_1_DIRECTION =
                DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction SHOT_2_DIRECTION =
                DcMotorSimple.Direction.REVERSE;

        // TeleOp velocity PIDF. Higher P/F helps the flywheels reach and hold target speed.
        public static final double PIDF_P = 37.5;
        public static final double PIDF_I = 0.0033;
        public static final double PIDF_D = 0.0002;
        public static final double PIDF_F = 19.5;
        public static final double AUTO_PIDF_P = 28.0;
        public static final double AUTO_PIDF_I = 0.0033;
        public static final double AUTO_PIDF_D = 0.0002;
        public static final double AUTO_PIDF_F = 14.5;
        public static final double VELOCITY_TOLERANCE_TICKS_PER_SECOND = 320.0;
        public static final double STABLE_TIME_SECONDS = 0.1;
        public static final boolean MAXIMIZE_CONFIGURED_MOTOR_RPM_FRACTION = true;
        public static boolean OPEN_LOOP_FULL_POWER_DIAGNOSTIC = false;
        public static final double OPEN_LOOP_DIAGNOSTIC_POWER = 1.0;

        private Shooter() {
        }
    }

    public static final class Feeder {

        // Placeholder servo values. Verify these with no balls loaded before using auto feed.
        public static final double STOPPER_CLOSED_POSITION = 0.6;
        public static final double STOPPER_OPEN_POSITION = 0.78;
        public static final double KICKER_REST_POSITION = 0.8;
        public static final double KICKER_FIRE_POSITION = 0.48;
        public static final double FEED_INTAKE_POWER = -0.7;
        public static final double MANUAL_INTAKE_POWER = -0.7;
        public static final double REVERSE_INTAKE_POWER = 0.85;
        public static final double VISION_READY_FAILSAFE_SECONDS = 0.40;

        private Feeder() {
        }
    }

    public static final class Vision {

        public static final int PIPELINE_INDEX = 0;
        public static final int POLL_RATE_HZ = 100;

        /*
         * The Limelight streams all match so the first correction is fast, but tx is only
         * allowed to affect turret aim while the shoot button is held.
         */
        public static final double MAX_STALENESS_MS = 160.0;
        public static final double MAX_TOTAL_LATENCY_MS = 200.0;
        public static final double TX_READY_TOLERANCE_DEGREES = 1.1;

        /*
         * Distance from target vertical angle requires the lens height and pitch.
         * Measure from floor to Limelight lens center and from horizontal to the camera.
         */
        public static final double GOAL_HEIGHT_INCHES = 32.67;
        public static final double LIMELIGHT_LENS_HEIGHT_INCHES = 12.74;
        public static final double LIMELIGHT_MOUNT_PITCH_DEGREES = 10.0;
        public static final double MIN_DISTANCE_INCHES = 10.0;
        public static final double MAX_DISTANCE_INCHES = 144.0;

        private Vision() {
        }
    }

    public static final class Ballistics {

        public static final double NOMINAL_BATTERY_VOLTAGE = 12.4;

        /*
         * If the camera crosshair or barrel is not perfectly aligned with turret zero,
         * adjust this. Positive means the shooter points CCW from the turret encoder angle.
         */
        public static final double SHOOTER_AXIS_OFFSET_DEGREES = 0.0;

        /*
         * Positive Limelight tx usually means target is to camera right. With robot-relative
         * angles positive CCW, a target to the right needs a clockwise correction, so -1.
         */
        public static final double LIMELIGHT_TX_TO_TURRET_DEGREES = -1.0;
        public static final double MAX_LIMELIGHT_FINE_CORRECTION_DEGREES = 25.0;
        public static final double LIMELIGHT_CORRECTION_FILTER_ALPHA = 0.35;

        /*
         * First-pass ball speed for moving-shot lead. This estimates flight time only;
         * it does not command shooter velocity.
         */
        public static final double ESTIMATED_BALL_SPEED_INCHES_PER_SECOND = 190.0;
        public static final double MIN_FLIGHT_TIME_SECONDS = 0.12;
        public static final double MAX_FLIGHT_TIME_SECONDS = 0.85;

        /*
         * Placeholder velocity map in encoder ticks/sec. Replace every row with real
         * distance-to-shooter-speed test data.
         */
        public static final InterpolatingTable.Point[] SHOT_MAP = {
                new InterpolatingTable.Point(15.6, 600),
                new InterpolatingTable.Point(88.6, 670),
                new InterpolatingTable.Point(108.9, 820),
                new InterpolatingTable.Point( 144.1,930)



        };

        /*
         * If setVelocity already holds perfectly across battery voltage, reduce this toward 0.
         * Healthy batteries do not reduce the target; below nominal voltage commands more ticks/sec.
         */
        public static final double VOLTAGE_COMPENSATION_STRENGTH = 0.95;
        public static final double MIN_VOLTAGE_SCALE = 1.0;
        public static final double MAX_VOLTAGE_SCALE = 1.16;

        /*
         * Positive radial velocity means the robot is moving toward the goal, so less wheel
         * speed may be needed. Tune from moving-shot testing.
         */
        public static final double TPS_PER_INCH_PER_SECOND_RADIAL_VELOCITY = 1.35;

        private Ballistics() {
        }
    }
}
