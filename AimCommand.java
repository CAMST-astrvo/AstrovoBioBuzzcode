package org.firstinspires.ftc.teamcode.astrovos.commands;

import org.firstinspires.ftc.teamcode.astrovos.subsystems.BallisticsSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.util.FieldCoordinates;
import org.firstinspires.ftc.teamcode.astrovos.util.LimelightTarget;
import org.firstinspires.ftc.teamcode.astrovos.util.ShotPlan;

public class AimCommand {

    private final LocalizationSubsystem localization;
    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final BallisticsSubsystem ballistics;

    public AimCommand(
            LocalizationSubsystem localization,
            TurretSubsystem turret,
            ShooterSubsystem shooter,
            HoodSubsystem hood,
            BallisticsSubsystem ballistics
    ) {

        this.localization = localization;
        this.turret = turret;
        this.shooter = shooter;
        this.hood = hood;
        this.ballistics = ballistics;
    }

    public ShotPlan execute(
            FieldCoordinates.AllianceGoal selectedGoal,
            LimelightTarget limelightTarget,
            double batteryVoltage,
            boolean shooting
    ) {

        ShotPlan plan =
                ballistics.calculate(
                        selectedGoal,
                        localization,
                        limelightTarget,
                        batteryVoltage,
                        shooting,
                        turret.getRobotRelativeDegrees()
                );

        turret.update(plan.turretTargetRobotDegrees);
        shooter.update(
                shooting
                        ? plan.targetShooterVelocity
                        : 0
        );
        hood.update(
                plan.distanceInches,
                plan.targetShooterVelocity,
                shooter.getAverageVelocity(),
                batteryVoltage,
                shooting
        );

        return plan;
    }
}
