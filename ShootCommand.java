package org.firstinspires.ftc.teamcode.astrovos.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.astrovos.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.util.Constants;
import org.firstinspires.ftc.teamcode.astrovos.util.ShotPlan;

public class ShootCommand {

    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final FeederSubsystem feeder;
    private final VisionSubsystem vision;
    private final ElapsedTime visionFailsafeTimer = new ElapsedTime();

    private boolean lastMechanismsReady;
    private boolean lastVisionReady;
    private boolean lastFailsafeReady;
    private double lastFailsafeElapsedSeconds;

    public ShootCommand(
            TurretSubsystem turret,
            ShooterSubsystem shooter,
            HoodSubsystem hood,
            FeederSubsystem feeder,
            VisionSubsystem vision
    ) {

        this.turret = turret;
        this.shooter = shooter;
        this.hood = hood;
        this.feeder = feeder;
        this.vision = vision;
        visionFailsafeTimer.reset();
    }

    public boolean execute(
            boolean shootHeld,
            ShotPlan shotPlan,
            boolean manualIntake,
            boolean reverseIntake,
            boolean kickerHeld
    ) {

        boolean mechanismsReady =
                shootHeld
                        && shotPlan.aimReachable
                        && turret.isReady()
                        && hood.isReady()
                        && shooter.isReady();

        if (!mechanismsReady) {
            visionFailsafeTimer.reset();
        }

        boolean visionReady =
                vision.isReadyToShoot();

        double failsafeElapsedSeconds =
                mechanismsReady
                        ? visionFailsafeTimer.seconds()
                        : 0.0;

        boolean failsafeReady =
                mechanismsReady
                        && failsafeElapsedSeconds
                        >= Constants.Feeder.VISION_READY_FAILSAFE_SECONDS;

        boolean readyToFeed =
                mechanismsReady
                        && (visionReady || failsafeReady);

        lastMechanismsReady = mechanismsReady;
        lastVisionReady = visionReady;
        lastFailsafeReady = failsafeReady;
        lastFailsafeElapsedSeconds = failsafeElapsedSeconds;

        feeder.update(
                readyToFeed,
                manualIntake,
                reverseIntake,
                kickerHeld
        );

        return readyToFeed;
    }

    public boolean areMechanismsReady() {

        return lastMechanismsReady;
    }

    public boolean isVisionReady() {

        return lastVisionReady;
    }

    public boolean isFailsafeReady() {

        return lastFailsafeReady;
    }

    public double getFailsafeElapsedSeconds() {

        return lastFailsafeElapsedSeconds;
    }

    public void resetReadiness() {

        visionFailsafeTimer.reset();
        lastMechanismsReady = false;
        lastVisionReady = false;
        lastFailsafeReady = false;
        lastFailsafeElapsedSeconds = 0.0;
    }
}
