package org.firstinspires.ftc.teamcode.astrovos.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.astrovos.util.Constants;

public class ShooterSubsystem {

    private final DcMotorEx shot1;
    private final DcMotorEx shot2;
    private final ElapsedTime stableTimer = new ElapsedTime();

    private double targetVelocity;
    private boolean wasWithinTolerance;
    private boolean openLoopDiagnosticActive;

    public ShooterSubsystem(HardwareMap hardwareMap) {

        shot1 =
                hardwareMap.get(
                        DcMotorEx.class,
                        Constants.Hardware.SHOT_1_NAME
                );

        shot2 =
                hardwareMap.get(
                        DcMotorEx.class,
                        Constants.Hardware.SHOT_2_NAME
                );

        configureMotor(
                shot1,
                Constants.Shooter.SHOT_1_DIRECTION
        );

        configureMotor(
                shot2,
                Constants.Shooter.SHOT_2_DIRECTION
        );

        stableTimer.reset();
    }

    public void update(double targetVelocity) {

        this.targetVelocity =
                Math.max(0, targetVelocity);

        if (this.targetVelocity <= 0) {
            shot1.setPower(0);
            shot2.setPower(0);
            wasWithinTolerance = false;
            stableTimer.reset();
            return;
        }

        if (Constants.Shooter.OPEN_LOOP_FULL_POWER_DIAGNOSTIC) {
            setOpenLoopDiagnosticActive(true);
            shot1.setPower(Constants.Shooter.OPEN_LOOP_DIAGNOSTIC_POWER);
            shot2.setPower(Constants.Shooter.OPEN_LOOP_DIAGNOSTIC_POWER);
            updateStableTimer();
            return;
        }

        setOpenLoopDiagnosticActive(false);
        shot1.setVelocity(this.targetVelocity);
        shot2.setVelocity(this.targetVelocity);
        updateStableTimer();
    }

    private void updateStableTimer() {

        boolean withinTolerance =
                Math.abs(getAverageVelocity() - this.targetVelocity)
                        <= Constants.Shooter.VELOCITY_TOLERANCE_TICKS_PER_SECOND;

        if (!withinTolerance || !wasWithinTolerance) {
            stableTimer.reset();
        }

        wasWithinTolerance = withinTolerance;
    }

    public double getTargetVelocity() {

        return targetVelocity;
    }
    public double getshot1velocity(){

        return shot1.getVelocity();
    }
    public double getshot2velocity(){

        return shot2.getVelocity();
    }

    public double getShot1Power() {

        return shot1.getPower();
    }

    public double getShot2Power() {

        return shot2.getPower();
    }

    public double getAverageVelocity() {

        return (Math.abs(shot1.getVelocity())
                + Math.abs(shot2.getVelocity())) / 2.0;
    }

    public boolean isReady() {

        return wasWithinTolerance
                && stableTimer.seconds() >= Constants.Shooter.STABLE_TIME_SECONDS;
    }

    public void stop() {

        update(0);
    }

    public void useAutonomousPidf() {

        setVelocityPidf(
                Constants.Shooter.AUTO_PIDF_P,
                Constants.Shooter.AUTO_PIDF_I,
                Constants.Shooter.AUTO_PIDF_D,
                Constants.Shooter.AUTO_PIDF_F
        );
    }

    private void configureMotor(
            DcMotorEx motor,
            DcMotorSimple.Direction direction
    ) {

        motor.setDirection(direction);
        maximizeConfiguredMotorSpeedIfEnabled(motor);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocityPIDFCoefficients(
                Constants.Shooter.PIDF_P,
                Constants.Shooter.PIDF_I,
                Constants.Shooter.PIDF_D,
                Constants.Shooter.PIDF_F
        );
    }

    private void maximizeConfiguredMotorSpeedIfEnabled(DcMotorEx motor) {

        if (!Constants.Shooter.MAXIMIZE_CONFIGURED_MOTOR_RPM_FRACTION) {
            return;
        }

        MotorConfigurationType motorType =
                motor.getMotorType().clone();

        motorType.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(motorType);
    }

    private void setOpenLoopDiagnosticActive(boolean active) {

        if (openLoopDiagnosticActive == active) {
            return;
        }

        openLoopDiagnosticActive = active;

        DcMotor.RunMode mode =
                active
                        ? DcMotor.RunMode.RUN_WITHOUT_ENCODER
                        : DcMotor.RunMode.RUN_USING_ENCODER;

        shot1.setMode(mode);
        shot2.setMode(mode);

        if (!active) {
            setVelocityPidf(
                    Constants.Shooter.PIDF_P,
                    Constants.Shooter.PIDF_I,
                    Constants.Shooter.PIDF_D,
                    Constants.Shooter.PIDF_F
            );
        }
    }

    private void setVelocityPidf(
            double p,
            double i,
            double d,
            double f
    ) {

        shot1.setVelocityPIDFCoefficients(p, i, d, f);
        shot2.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public String getShot1MotorTypeName() {

        return shot1.getMotorType().getName();
    }

    public String getShot2MotorTypeName() {

        return shot2.getMotorType().getName();
    }

    public double getShot1ConfiguredMaxTicksPerSecond() {

        return shot1.getMotorType().getAchieveableMaxTicksPerSecond();
    }

    public double getShot2ConfiguredMaxTicksPerSecond() {

        return shot2.getMotorType().getAchieveableMaxTicksPerSecond();
    }

    public double getShot1TicksPerRev() {

        return shot1.getMotorType().getTicksPerRev();
    }

    public double getShot2TicksPerRev() {

        return shot2.getMotorType().getTicksPerRev();
    }

    public double getShot1MaxRpm() {

        return shot1.getMotorType().getMaxRPM();
    }

    public double getShot2MaxRpm() {

        return shot2.getMotorType().getMaxRPM();
    }

    public double getShot1AchievableMaxRpmFraction() {

        return shot1.getMotorType().getAchieveableMaxRPMFraction();
    }

    public double getShot2AchievableMaxRpmFraction() {

        return shot2.getMotorType().getAchieveableMaxRPMFraction();
    }

    public PIDFCoefficients getShot1VelocityPidf() {

        return shot1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isOpenLoopDiagnosticActive() {

        return openLoopDiagnosticActive;
    }
}
