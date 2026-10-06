package org.firstinspires.ftc.teamcode.astrovos.subsystems;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.astrovos.util.Constants;
import org.firstinspires.ftc.teamcode.astrovos.util.FieldCoordinates;
import org.firstinspires.ftc.teamcode.astrovos.util.InterpolatingTable;
import org.firstinspires.ftc.teamcode.astrovos.util.LimelightTarget;
import org.firstinspires.ftc.teamcode.astrovos.util.MathUtil;
import org.firstinspires.ftc.teamcode.astrovos.util.ShotPlan;

public class BallisticsSubsystem {

    private final InterpolatingTable shotMap =
            new InterpolatingTable(Constants.Ballistics.SHOT_MAP);
    private double filteredLimelightCorrectionDegrees;

    public ShotPlan calculate(
            FieldCoordinates.AllianceGoal selectedGoal,
            LocalizationSubsystem localization,
            LimelightTarget limelightTarget,
            double batteryVoltage,
            boolean shooting,
            double currentTurretRobotDegrees
    ) {

        return calculateFromPose(
                selectedGoal,
                localization.getXInches(),
                localization.getYInches(),
                localization.getHeadingRadians(),
                localization.getVelocityXInchesPerSecond(),
                localization.getVelocityYInchesPerSecond(),
                limelightTarget,
                batteryVoltage,
                shooting,
                currentTurretRobotDegrees
        );
    }

    public ShotPlan calculateFromPose(
            FieldCoordinates.AllianceGoal selectedGoal,
            double robotXInches,
            double robotYInches,
            double robotHeadingRadians,
            double robotVelocityXInchesPerSecond,
            double robotVelocityYInchesPerSecond,
            LimelightTarget limelightTarget,
            double batteryVoltage,
            boolean shooting,
            double currentTurretRobotDegrees
    ) {

        double dx =
                selectedGoal.xInches - robotXInches;

        double dy =
                selectedGoal.yInches - robotYInches;

        double odometryDistance =
                Math.hypot(dx, dy);

        double flightTime =
                Range.clip(
                        odometryDistance
                                / Constants.Ballistics.ESTIMATED_BALL_SPEED_INCHES_PER_SECOND,
                        Constants.Ballistics.MIN_FLIGHT_TIME_SECONDS,
                        Constants.Ballistics.MAX_FLIGHT_TIME_SECONDS
                );

        double leadX =
                dx - (robotVelocityXInchesPerSecond * flightTime);

        double leadY =
                dy - (robotVelocityYInchesPerSecond * flightTime);

        double targetFieldBearing =
                Math.atan2(
                        leadY,
                        leadX
                );

        double baseTargetRobotDegrees =
                MathUtil.normalizeDegrees(
                        Math.toDegrees(
                                targetFieldBearing
                                        - robotHeadingRadians
                        )
                                - Constants.Ballistics.SHOOTER_AXIS_OFFSET_DEGREES
                );

        double targetRobotDegrees =
                baseTargetRobotDegrees;

        double rawLimelightCorrectionDegrees =
                0.0;

        boolean usingLimelightCorrection =
                false;

        if (shooting && limelightTarget.valid) {
            double fineCorrection =
                    Range.clip(
                            limelightTarget.txDegrees
                                    * Constants.Ballistics.LIMELIGHT_TX_TO_TURRET_DEGREES,
                            -Constants.Ballistics.MAX_LIMELIGHT_FINE_CORRECTION_DEGREES,
                            Constants.Ballistics.MAX_LIMELIGHT_FINE_CORRECTION_DEGREES
                    );

            double visionTargetRobotDegrees =
                    MathUtil.normalizeDegrees(
                            currentTurretRobotDegrees + fineCorrection
                    );

            rawLimelightCorrectionDegrees =
                    Range.clip(
                            MathUtil.normalizeDegrees(
                                    visionTargetRobotDegrees - baseTargetRobotDegrees
                            ),
                            -Constants.Ballistics.MAX_LIMELIGHT_FINE_CORRECTION_DEGREES,
                            Constants.Ballistics.MAX_LIMELIGHT_FINE_CORRECTION_DEGREES
                    );

            filteredLimelightCorrectionDegrees =
                    blend(
                            filteredLimelightCorrectionDegrees,
                            rawLimelightCorrectionDegrees,
                            Constants.Ballistics.LIMELIGHT_CORRECTION_FILTER_ALPHA
                    );

            targetRobotDegrees =
                    MathUtil.normalizeDegrees(
                            baseTargetRobotDegrees + filteredLimelightCorrectionDegrees
                    );

            usingLimelightCorrection = true;
        } else if (shooting) {
            filteredLimelightCorrectionDegrees =
                    blend(
                            filteredLimelightCorrectionDegrees,
                            0.0,
                            Constants.Ballistics.LIMELIGHT_CORRECTION_FILTER_ALPHA
                    );

            targetRobotDegrees =
                    MathUtil.normalizeDegrees(
                            baseTargetRobotDegrees + filteredLimelightCorrectionDegrees
                    );

            usingLimelightCorrection =
                    Math.abs(filteredLimelightCorrectionDegrees) > 0.01;
        } else {
            filteredLimelightCorrectionDegrees = 0.0;
        }

        double distanceForShot =
                shooting
                        && limelightTarget.valid
                        && limelightTarget.distanceValid
                        ? limelightTarget.distanceInches
                        : odometryDistance;

        double unitX =
                odometryDistance > 0.001
                        ? dx / odometryDistance
                        : 0;

        double unitY =
                odometryDistance > 0.001
                        ? dy / odometryDistance
                        : 0;

        double radialVelocity =
                (robotVelocityXInchesPerSecond * unitX)
                        + (robotVelocityYInchesPerSecond * unitY);

        double baseVelocity =
                shotMap.get(distanceForShot);

        double voltageScale =
                1.0;

        if (batteryVoltage < Constants.Ballistics.NOMINAL_BATTERY_VOLTAGE) {
            voltageScale =
                    1.0
                            + ((Constants.Ballistics.NOMINAL_BATTERY_VOLTAGE
                            / Math.max(8.0, batteryVoltage))
                            - 1.0)
                            * Constants.Ballistics.VOLTAGE_COMPENSATION_STRENGTH;
        }

        voltageScale =
                Range.clip(
                        voltageScale,
                        Math.max(1.0, Constants.Ballistics.MIN_VOLTAGE_SCALE),
                        Constants.Ballistics.MAX_VOLTAGE_SCALE
                );

        double movingShotCompensation =
                -radialVelocity
                        * Constants.Ballistics.TPS_PER_INCH_PER_SECOND_RADIAL_VELOCITY;

        ShotPlan plan =
                new ShotPlan();

        plan.turretTargetRobotDegrees = targetRobotDegrees;
        plan.odometryDistanceInches = odometryDistance;
        plan.distanceInches = distanceForShot;
        plan.baseShooterVelocity = baseVelocity;
        plan.voltageScale = voltageScale;
        plan.movingShotVelocityCompensation = movingShotCompensation;
        plan.targetShooterVelocity =
                Math.max(
                        0,
                        (baseVelocity * voltageScale)
                                + movingShotCompensation
                );
        plan.flightTimeSeconds = flightTime;
        plan.rawLimelightCorrectionDegrees =
                rawLimelightCorrectionDegrees;
        plan.filteredLimelightCorrectionDegrees =
                usingLimelightCorrection
                        ? filteredLimelightCorrectionDegrees
                        : 0.0;
        plan.usingLimelightDistance =
                shooting
                        && limelightTarget.valid
                        && limelightTarget.distanceValid;
        plan.usingLimelightCorrection =
                usingLimelightCorrection;
        plan.aimReachable =
                targetRobotDegrees >= Constants.Turret.MIN_ROBOT_RELATIVE_DEGREES
                        && targetRobotDegrees <= Constants.Turret.MAX_ROBOT_RELATIVE_DEGREES;

        return plan;
    }

    private double blend(
            double oldValue,
            double newValue,
            double alpha
    ) {

        double clippedAlpha =
                Range.clip(alpha, 0.0, 1.0);

        return (oldValue * (1.0 - clippedAlpha))
                + (newValue * clippedAlpha);
    }
}
