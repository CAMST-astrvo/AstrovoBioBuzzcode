package org.firstinspires.ftc.teamcode.astrovos.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.astrovos.util.Constants;
import org.firstinspires.ftc.teamcode.astrovos.util.InterpolatingTable;

public class HoodSubsystem {

    private final Servo hood;
    private final InterpolatingTable hoodMap =
            new InterpolatingTable(Constants.Hood.HOOD_MAP);
    private final ElapsedTime settleTimer = new ElapsedTime();

    private double targetPosition =
            Constants.Hood.HOOD_PARK_POSITION;

    public HoodSubsystem(HardwareMap hardwareMap) {

        hood =
                hardwareMap.get(
                        Servo.class,
                        Constants.Hardware.HOOD_SERVO_NAME
                );

        hood.setPosition(Constants.Hood.HOOD_PARK_POSITION);
        settleTimer.reset();
    }

    public void update(
            double distanceInches,
            double targetVelocity,
            double actualVelocity,
            double batteryVoltage,
            boolean shooting
    ) {

        if (!shooting) {
            holdParkPosition();
            return;
        }

        double basePosition =
                hoodMap.get(distanceInches);

        double voltageBias =
                Math.max(0, Constants.Ballistics.NOMINAL_BATTERY_VOLTAGE - batteryVoltage)
                        * Constants.Hood.HOOD_POSITION_PER_LOW_VOLT;

        double velocityDropBias =
                Math.max(0, targetVelocity - actualVelocity)
                        / 100.0
                        * Constants.Hood.HOOD_POSITION_PER_100_TPS_LOW;

        setTarget(
                Range.clip(
                        basePosition + voltageBias + velocityDropBias,
                        Constants.Hood.HOOD_MIN_POSITION,
                        Constants.Hood.HOOD_MAX_POSITION
                )
        );
    }

    public void holdParkPosition() {

        setTarget(Constants.Hood.HOOD_PARK_POSITION);
    }

    public boolean isReady() {

        return settleTimer.seconds()
                >= Constants.Hood.SETTLE_TIME_SECONDS;
    }

    public double getTargetPosition() {

        return targetPosition;
    }

    public void stop() {

        holdParkPosition();
    }

    private void setTarget(double targetPosition) {

        double clipped =
                Range.clip(
                        targetPosition,
                        Constants.Hood.HOOD_MIN_POSITION,
                        Constants.Hood.HOOD_MAX_POSITION
                );

        if (Math.abs(clipped - this.targetPosition)
                > Constants.Hood.TARGET_CHANGE_EPSILON) {
            settleTimer.reset();
        }

        this.targetPosition = clipped;
        hood.setPosition(clipped);
    }
}
