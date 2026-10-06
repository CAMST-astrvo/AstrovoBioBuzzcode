package org.firstinspires.ftc.teamcode.astrovos.commands;

import org.firstinspires.ftc.teamcode.astrovos.subsystems.BallisticsSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.util.FieldCoordinates;
import org.firstinspires.ftc.teamcode.astrovos.util.LimelightTarget;
import org.firstinspires.ftc.teamcode.astrovos.util.ShotPlan;

public class TrackGoalCommand {

    private final LocalizationSubsystem localization;
    private final TurretSubsystem turret;
    private final BallisticsSubsystem ballistics;

    public TrackGoalCommand(
            LocalizationSubsystem localization,
            TurretSubsystem turret,
            BallisticsSubsystem ballistics
    ) {

        this.localization = localization;
        this.turret = turret;
        this.ballistics = ballistics;
    }

    public ShotPlan execute(
            FieldCoordinates.AllianceGoal selectedGoal,
            double batteryVoltage
    ) {

        ShotPlan plan =
                ballistics.calculate(
                        selectedGoal,
                        localization,
                        LimelightTarget.inactive("tracking with odometry"),
                        batteryVoltage,
                        false,
                        turret.getRobotRelativeDegrees()
                );

        turret.update(plan.turretTargetRobotDegrees);
        return plan;
    }
}
