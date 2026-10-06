package org.firstinspires.ftc.teamcode.astrovos.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.astrovos.util.Constants;

public class TurretSubsystem {

    private final DcMotorEx motor;
    private final TouchSensor homeSwitch;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime homeDebounceTimer = new ElapsedTime();
    private final ElapsedTime homeSearchTimer = new ElapsedTime();
    private final ElapsedTime homeStallTimer = new ElapsedTime();

    private double lastTargetRobotDegrees =
            Constants.Turret.HOME_ROBOT_RELATIVE_DEGREES;

    private double lastErrorDegrees;
    private double lastErrorRateDegreesPerSecond;
    private double lastMotorPower;
    private String faultReason = "none";
    private int homeStallCheckTicks;
    private boolean lastTargetClamped;
    private boolean outsideGuard;
    private boolean homed;
    private boolean faulted;
    private boolean homeSwitchPressed;
    private boolean lastRawHomeSwitchPressed;
    private boolean homeSearchActive;
    private boolean homeRecoveryActive;
    private boolean homeRecoveryFaulted;

    public TurretSubsystem(HardwareMap hardwareMap) {

        motor =
                hardwareMap.get(
                        DcMotorEx.class,
                        Constants.Hardware.TURRET_NAME
                );

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        homeSwitch =
                hardwareMap.get(
                        TouchSensor.class,
                        Constants.Hardware.TURRET_HOME_SWITCH_NAME
                );

        lastRawHomeSwitchPressed = readRawHomeSwitchPressed();

        timer.reset();
        homeDebounceTimer.reset();
        homeSearchTimer.reset();
        homeStallTimer.reset();
    }

    public void update(double requestedRobotDegrees) {

        updateHomeSwitch();

        if (homeRecoveryActive) {
            setMotorPower(0);
            return;
        }

        if (!homed) {
            runHomeSearch();
            return;
        }

        if (faulted) {
            setMotorPower(0);
            return;
        }

        double currentDegrees =
                getRobotRelativeDegrees();

        double targetDegrees =
                resolveReachableTarget(requestedRobotDegrees, currentDegrees);

        lastTargetClamped =
                Double.isNaN(targetDegrees);

        if (lastTargetClamped) {
            lastTargetRobotDegrees = requestedRobotDegrees;
            lastErrorDegrees = 0;
            lastErrorRateDegreesPerSecond = 0;
            homeSearchActive = false;
            setMotorPower(0);
            return;
        }

        if (shouldSearchForHome(targetDegrees)) {
            runHomeSearch();
            return;
        }

        homeSearchActive = false;

        outsideGuard =
                currentDegrees < Constants.Turret.MIN_ROBOT_RELATIVE_DEGREES
                - Constants.Turret.LIMIT_GUARD_DEGREES
                        || currentDegrees > Constants.Turret.MAX_ROBOT_RELATIVE_DEGREES
                        + Constants.Turret.LIMIT_GUARD_DEGREES;

        if (outsideGuard) {
            faulted = true;
            faultReason = "outside turret guard";
            setMotorPower(0);
            return;
        }

        lastTargetRobotDegrees =
                targetDegrees;

        double errorDegrees =
                targetDegrees - currentDegrees;

        double dt =
                timer.seconds();

        timer.reset();

        if (dt <= 0 || dt > 0.25) {
            dt = 0.02;
        }

        double errorRate =
                (errorDegrees - lastErrorDegrees) / dt;

        lastErrorDegrees =
                errorDegrees;

        lastErrorRateDegreesPerSecond =
                errorRate;

        double power =
                Constants.Turret.POWER_SIGN
                        * ((Constants.Turret.KP * errorDegrees)
                        + (Constants.Turret.KD * errorRate));

        if (Math.abs(errorDegrees)
                <= Constants.Turret.READY_TOLERANCE_DEGREES) {
            power =
                    Range.clip(
                            power,
                            -Constants.Turret.HOLD_POWER_MAX,
                            Constants.Turret.HOLD_POWER_MAX
                    );
        }

        power =
                Range.clip(
                        power,
                        -Constants.Turret.MAX_POWER,
                        Constants.Turret.MAX_POWER
                );

        double angleDirection =
                power * Constants.Turret.POWER_SIGN;

        if (currentDegrees <= Constants.Turret.MIN_ROBOT_RELATIVE_DEGREES
                && angleDirection < 0) {
            power = 0;
        }

        if (currentDegrees >= Constants.Turret.MAX_ROBOT_RELATIVE_DEGREES
                && angleDirection > 0) {
            power = 0;
        }

        setMotorPower(power);
    }

    public void updateHoming(boolean allowMotion) {

        updateHomeSwitch();

        if (!homed && !faulted && allowMotion) {
            runHomeSearch();
        } else {
            homeSearchActive = false;
            setMotorPower(0);
        }
    }

    public void requestHomeRecovery() {

        updateHomeSwitch();

        if (homeSwitchPressed) {
            homeRecoveryActive = false;
            homeSearchActive = false;
            setMotorPower(0);
            return;
        }

        if (homeRecoveryFaulted) {
            setMotorPower(0);
            return;
        }

        if (!homeRecoveryActive) {
            homeRecoveryActive = true;
            homeSearchActive = false;
            faulted = false;
            faultReason = "none";
            outsideGuard = false;
        }
    }

    public void cancelHomeRecovery() {

        homeRecoveryActive = false;
        homeRecoveryFaulted = false;
        homeSearchActive = false;
        setMotorPower(0);
    }

    public void updateHomeRecovery(boolean allowMotion) {

        updateHomeSwitch();

        if (!homeRecoveryActive) {
            setMotorPower(0);
            return;
        }

        if (homeSwitchPressed) {
            homeRecoveryActive = false;
            homeSearchActive = false;
            setMotorPower(0);
            return;
        }

        if (!allowMotion || faulted) {
            if (faulted) {
                homeRecoveryActive = false;
                homeRecoveryFaulted = true;
            }

            homeSearchActive = false;
            setMotorPower(0);
            return;
        }

        runHomeSearch();

        if (faulted) {
            homeRecoveryActive = false;
            homeRecoveryFaulted = true;
        }
    }

    private void updateHomeSwitch() {

        boolean rawPressed =
                readRawHomeSwitchPressed();

        lastRawHomeSwitchPressed = rawPressed;

        if (rawPressed) {
            setMotorPower(0);
            homeDebounceTimer.reset();

            if (!homeSwitchPressed) {
                homeSwitchPressed = true;
                captureHome();
            }

            return;
        }

        if (homeSwitchPressed
                && homeDebounceTimer.seconds()
                >= Constants.Turret.HOME_SWITCH_DEBOUNCE_SECONDS) {
            homeSwitchPressed = false;
        }
    }

    private boolean readRawHomeSwitchPressed() {

        return homeSwitch.isPressed();
    }

    private void captureHome() {

        setMotorPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        homed = true;
        faulted = false;
        faultReason = "none";
        homeSearchActive = false;
        homeRecoveryActive = false;
        homeRecoveryFaulted = false;
        outsideGuard = false;
        lastTargetRobotDegrees =
                Constants.Turret.HOME_ROBOT_RELATIVE_DEGREES;
        lastErrorDegrees = 0;
        lastErrorRateDegreesPerSecond = 0;
        timer.reset();
    }

    private void runHomeSearch() {

        if (homeSwitchPressed) {
            homeSearchActive = false;
            setMotorPower(0);
            return;
        }

        if (!homeSearchActive) {
            homeSearchActive = true;
            homeSearchTimer.reset();
            homeStallCheckTicks = motor.getCurrentPosition();
            homeStallTimer.reset();
        }

        if (homeSearchTimer.seconds()
                >= Constants.Turret.HOME_SEARCH_TIMEOUT_SECONDS) {
            faulted = true;
            faultReason = "home search timeout";
            homeSearchActive = false;
            setMotorPower(0);
            return;
        }

        if (homeStallTimer.seconds()
                >= Constants.Turret.HOME_STALL_CHECK_SECONDS) {
            double movementDegrees =
                    Math.abs(motor.getCurrentPosition() - homeStallCheckTicks)
                            / Constants.Turret.TICKS_PER_TURRET_DEGREE;

            if (movementDegrees
                    < Constants.Turret.HOME_STALL_MIN_MOVEMENT_DEGREES) {
                faulted = true;
                faultReason = "home search stalled";
                homeSearchActive = false;
                setMotorPower(0);
                return;
            }

            homeStallCheckTicks = motor.getCurrentPosition();
            homeStallTimer.reset();
        }

        outsideGuard = true;
        setMotorPower(
                Constants.Turret.HOME_SEARCH_ANGLE_DIRECTION
                        * Constants.Turret.POWER_SIGN
                        * Constants.Turret.HOME_SEARCH_POWER
        );
    }

    private void setMotorPower(double requestedPower) {

        boolean rawPressed =
                readRawHomeSwitchPressed();

        lastRawHomeSwitchPressed = rawPressed;

        double angleDirection =
                requestedPower * Constants.Turret.POWER_SIGN;

        if (rawPressed && angleDirection < 0) {
            requestedPower = 0;
        }

        lastMotorPower = requestedPower;
        motor.setPower(requestedPower);
    }

    private boolean shouldSearchForHome(double targetDegrees) {

        return !homeSwitchPressed
                && targetDegrees
                <= Constants.Turret.MIN_ROBOT_RELATIVE_DEGREES
                + Constants.Turret.HOME_TARGET_TOLERANCE_DEGREES;
    }

    private double resolveReachableTarget(
            double requestedDegrees,
            double currentDegrees
    ) {

        double bestTarget = Double.NaN;
        double bestDistance = Double.POSITIVE_INFINITY;

        for (int turns = -1; turns <= 1; turns++) {
            double candidate =
                    requestedDegrees + (turns * 360.0);

            if (candidate >= Constants.Turret.MIN_ROBOT_RELATIVE_DEGREES
                    && candidate <= Constants.Turret.MAX_ROBOT_RELATIVE_DEGREES) {
                double distance =
                        Math.abs(candidate - currentDegrees);

                if (distance < bestDistance) {
                    bestTarget = candidate;
                    bestDistance = distance;
                }
            }
        }

        return bestTarget;
    }

    public double getRobotRelativeDegrees() {

        return Constants.Turret.HOME_ROBOT_RELATIVE_DEGREES
                + getTravelDegrees();
    }

    public double getTravelDegrees() {

        return motor.getCurrentPosition()
                * Constants.Turret.ENCODER_TICK_TO_DEGREES_SIGN
                / Constants.Turret.TICKS_PER_TURRET_DEGREE;
    }

    public double getLastTargetRobotDegrees() {

        return lastTargetRobotDegrees;
    }

    public double getLastErrorDegrees() {

        return lastErrorDegrees;
    }

    public boolean wasLastTargetClamped() {

        return lastTargetClamped;
    }

    public boolean isReady() {

        return isSafe()
                && !lastTargetClamped
                && Math.abs(lastErrorDegrees)
                <= Constants.Turret.READY_TOLERANCE_DEGREES
                && Math.abs(lastErrorRateDegreesPerSecond)
                <= Constants.Turret.READY_ERROR_RATE_DEGREES_PER_SECOND;
    }

    public boolean isSafe() {

        return homed
                && !faulted
                && !homeSearchActive
                && !homeRecoveryActive
                && !outsideGuard
                && (!lastRawHomeSwitchPressed || homeSwitchPressed);
    }

    public boolean isHomed() {

        return homed;
    }

    public boolean isHomeSwitchPressed() {

        return homeSwitchPressed;
    }

    public boolean isRawHomeSensorPressed() {

        return lastRawHomeSwitchPressed;
    }

    public boolean isHoming() {

        return homeSearchActive;
    }

    public boolean isHomeRecoveryActive() {

        return homeRecoveryActive;
    }

    public boolean isFaulted() {

        return faulted;
    }

    public double getLastMotorPower() {

        return lastMotorPower;
    }

    public String getFaultReason() {

        return faultReason;
    }

    public void stop() {

        setMotorPower(0);
    }
}
