package org.firstinspires.ftc.teamcode.AutoHelpers;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.astrovos.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.BallisticsSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.util.Constants;
import org.firstinspires.ftc.teamcode.astrovos.util.FieldCoordinates;
import org.firstinspires.ftc.teamcode.astrovos.util.LimelightTarget;
import org.firstinspires.ftc.teamcode.astrovos.util.ShotPlan;

@Configurable
public class AutoShotController {

    public static double FEED_SECONDS = 2.8;
    public static double END_KICKER_SECONDS = 0.35;
    public static double RESET_SECONDS = 0.20;
    public static double MAX_PREPARE_SECONDS = 10.0;
    public static double AUTO_FORCE_FEED_AFTER_SECONDS = 2.5;
    public static double MIN_FORCE_FEED_SHOOTER_VELOCITY = 1600.0;
    public static double CLOSE_AUTO_SHOOTER_VELOCITY_BOOST = 25.0;
    public static double CLOSE_AUTO_EXTRA_SPINUP_SECONDS = 0.30;
    public static double CLOSE_AUTO_EXTRA_FEED_SECONDS = 1.0;
    public static double FAR_FIRST_SHOT_VELOCITY_OFFSET = -75.0;
    public static boolean FIRE_KICKER_DURING_FEED = false;
    public static boolean FIRE_KICKER_AT_END = true;
    public static boolean ALLOW_AUTO_FORCE_FEED = true;

    private enum AutoShotState {
        IDLE,
        PREPARE,
        FEED,
        RESET
    }

    private Follower follower;
    private HardwareMap hardwareMap;
    private FieldCoordinates.AllianceGoal selectedGoal;
    private TurretSubsystem turret;
    private ShooterSubsystem shooter;
    private HoodSubsystem hood;
    private FeederSubsystem feeder;
    private VisionSubsystem vision;
    private BallisticsSubsystem ballistics;
    private ShootCommand shootCommand;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime poseTimer = new ElapsedTime();

    private AutoShotState state = AutoShotState.IDLE;
    private Pose lastPose;
    private double velocityXInchesPerSecond;
    private double velocityYInchesPerSecond;
    private int shotsRemaining;
    private boolean lastReadyToFeed;
    private boolean lastTimedOut;
    private double lastBatteryVoltage = Constants.Ballistics.NOMINAL_BATTERY_VOLTAGE;
    private String lastStatus = "idle";
    private String lastFeedBlocker = "idle";
    private LimelightTarget lastTarget = LimelightTarget.inactive("not sampled");
    private ShotPlan lastShotPlan = new ShotPlan();
    private boolean intakeEnabled;
    private double shooterVelocityBoost;
    private double activeShotVelocityOffset;
    private double minimumPrepareSeconds;
    private double extraFeedSeconds;

    public void init(
            HardwareMap hardwareMap,
            Follower follower,
            FieldCoordinates.AllianceGoal selectedGoal
    ) {

        this.hardwareMap = hardwareMap;
        this.follower = follower;
        this.selectedGoal = selectedGoal;

        turret = new TurretSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        shooter.useAutonomousPidf();
        hood = new HoodSubsystem(hardwareMap);
        feeder = new FeederSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        ballistics = new BallisticsSubsystem();
        shootCommand =
                new ShootCommand(
                        turret,
                        shooter,
                        hood,
                        feeder,
                        vision
                );

        stateTimer.reset();
        poseTimer.reset();
        capturePoseSnapshot();
    }

    public void fireShots(int numberOfShots) {

        fireShotsWithVelocityOffset(numberOfShots, 0.0);
    }

    public void fireShotsWithVelocityOffset(
            int numberOfShots,
            double velocityOffset
    ) {

        if (state != AutoShotState.IDLE || numberOfShots <= 0) {
            return;
        }

        shotsRemaining = numberOfShots;
        activeShotVelocityOffset = velocityOffset;
        intakeEnabled = true;
        lastTimedOut = false;
        lastStatus = "preparing";
        lastFeedBlocker = "starting shot";
        shootCommand.resetReadiness();
        state = AutoShotState.PREPARE;
        stateTimer.reset();
    }

    public void update() {

        updatePoseVelocity();

        if (state == AutoShotState.IDLE) {
            updateIdleAiming();
            return;
        }

        updateShootingMechanisms();

        switch (state) {
            case PREPARE:
                runPrepareState();
                break;

            case FEED:
                runFeedState();
                break;

            case RESET:
                runResetState();
                break;

            case IDLE:
            default:
                break;
        }
    }

    public boolean isBusy() {

        return state != AutoShotState.IDLE;
    }

    public void setIntakeEnabled(boolean intakeEnabled) {

        this.intakeEnabled = intakeEnabled;
    }

    public void setSelectedGoal(FieldCoordinates.AllianceGoal selectedGoal) {

        this.selectedGoal = selectedGoal;
    }

    public void useCloseAutoShooterSpeed() {

        shooterVelocityBoost =
                CLOSE_AUTO_SHOOTER_VELOCITY_BOOST;
        minimumPrepareSeconds =
                CLOSE_AUTO_EXTRA_SPINUP_SECONDS;
        extraFeedSeconds =
                CLOSE_AUTO_EXTRA_FEED_SECONDS;
    }

    public boolean isIntakeEnabled() {

        return intakeEnabled;
    }

    public double getVelocityReading() {

        return shooter == null
                ? 0.0
                : shooter.getAverageVelocity();
    }

    public String getStateName() {

        return state.name().toLowerCase();
    }

    public String getLastStatus() {

        return lastStatus;
    }

    public boolean wasLastReadyToFeed() {

        return lastReadyToFeed;
    }

    public boolean didLastShotTimeOut() {

        return lastTimedOut;
    }

    public ShotPlan getLastShotPlan() {

        return lastShotPlan;
    }

    public LimelightTarget getLastTarget() {

        return lastTarget;
    }

    public boolean isTurretReady() {

        return turret != null && turret.isReady();
    }

    public boolean isHoodReady() {

        return hood != null && hood.isReady();
    }

    public boolean isShooterReady() {

        return shooter != null && shooter.isReady();
    }

    public boolean isVisionReady() {

        return shootCommand != null && shootCommand.isVisionReady();
    }

    public boolean areMechanismsReady() {

        return shootCommand != null && shootCommand.areMechanismsReady();
    }

    public boolean isFailsafeReady() {

        return shootCommand != null && shootCommand.isFailsafeReady();
    }

    public double getFailsafeElapsedSeconds() {

        return shootCommand == null
                ? 0.0
                : shootCommand.getFailsafeElapsedSeconds();
    }

    public boolean isTurretSafe() {

        return turret != null && turret.isSafe();
    }

    public boolean isTurretHomed() {

        return turret != null && turret.isHomed();
    }

    public String getTurretFaultReason() {

        return turret == null
                ? "turret missing"
                : turret.getFaultReason();
    }

    public String getLastFeedBlocker() {

        return lastFeedBlocker;
    }

    public double getTargetVelocity() {

        return shooter == null
                ? 0.0
                : shooter.getTargetVelocity();
    }

    public double getBatteryVoltage() {

        return lastBatteryVoltage;
    }

    public void stop() {

        if (turret != null) {
            turret.stop();
        }

        if (shooter != null) {
            shooter.stop();
        }

        if (hood != null) {
            hood.stop();
        }

        if (feeder != null) {
            feeder.stop();
        }

        if (vision != null) {
            vision.stop();
        }
    }

    private void runPrepareState() {

        lastReadyToFeed =
                shootCommand.execute(
                        true,
                        lastShotPlan,
                        intakeEnabled,
                        false,
                        false
                );

        if (lastReadyToFeed) {
            if (stateTimer.seconds() < minimumPrepareSeconds) {
                lastStatus = "preparing: extra spinup";
                lastFeedBlocker = "extra spinup";
                return;
            }

            beginFeed("feeding: normal gate ready");
            return;
        }

        if (shouldForceAutoFeed()) {
            lastTimedOut = true;
            beginFeed("feeding: auto velocity fallback");
            return;
        }

        if (stateTimer.seconds() >= MAX_PREPARE_SECONDS) {
            lastTimedOut = true;
            lastStatus = "still waiting: " + getCurrentFeedBlocker();
        } else {
            lastStatus = "preparing: " + getCurrentFeedBlocker();
        }
    }

    private void runFeedState() {

        double feedElapsedSeconds =
                stateTimer.seconds();

        feeder.update(
                true,
                false,
                false,
                shouldFireKicker(feedElapsedSeconds)
        );

        lastReadyToFeed = true;

        if (feedElapsedSeconds < getFeedSeconds()) {
            return;
        }

        shotsRemaining--;

        if (shotsRemaining > 0) {
            lastStatus = "resetting";
            lastFeedBlocker = "resetting";
            feeder.update(false, false, false, false);
            shootCommand.resetReadiness();
            state = AutoShotState.RESET;
            stateTimer.reset();
        } else {
            lastStatus = "complete";
            finishShotSequence();
        }
    }

    private void runResetState() {

        feeder.update(false, false, false, false);

        if (stateTimer.seconds() < RESET_SECONDS) {
            return;
        }

        lastStatus = "preparing";
        lastFeedBlocker = "starting next shot";
        shootCommand.resetReadiness();
        state = AutoShotState.PREPARE;
        stateTimer.reset();
    }

    private void updateIdleAiming() {

        vision.update(false, selectedGoal);
        lastTarget = vision.getLastTarget();
        lastBatteryVoltage = readBatteryVoltage();
        lastShotPlan =
                calculateShotPlan(false);

        turret.update(lastShotPlan.turretTargetRobotDegrees);
        shooter.update(0);
        hood.holdParkPosition();
        feeder.update(false, intakeEnabled, false, false);
        lastReadyToFeed = false;
        lastFeedBlocker = "idle";
    }

    private void updateShootingMechanisms() {

        vision.update(true, selectedGoal);
        lastTarget = vision.getLastTarget();
        lastBatteryVoltage = readBatteryVoltage();
        lastShotPlan =
                calculateShotPlan(true);

        turret.update(lastShotPlan.turretTargetRobotDegrees);
        shooter.update(lastShotPlan.targetShooterVelocity);
        hood.update(
                lastShotPlan.distanceInches,
                lastShotPlan.targetShooterVelocity,
                shooter.getAverageVelocity(),
                lastBatteryVoltage,
                true
        );
    }

    private void beginFeed(String status) {

        lastStatus = status;
        lastFeedBlocker = "none";
        state = AutoShotState.FEED;
        stateTimer.reset();
        feeder.update(
                true,
                false,
                false,
                shouldFireKicker(0.0)
        );
    }

    private boolean shouldFireKicker(double feedElapsedSeconds) {

        double endKickerStartSeconds =
                Math.max(
                        0.0,
                        getFeedSeconds() - END_KICKER_SECONDS
                );

        return FIRE_KICKER_DURING_FEED
                || (FIRE_KICKER_AT_END
                && feedElapsedSeconds >= endKickerStartSeconds);
    }

    private double getFeedSeconds() {

        return FEED_SECONDS + extraFeedSeconds;
    }

    private boolean shouldForceAutoFeed() {

        return ALLOW_AUTO_FORCE_FEED
                && stateTimer.seconds() >= AUTO_FORCE_FEED_AFTER_SECONDS
                && lastShotPlan.aimReachable
                && turret.isReady()
                && hood.isReady()
                && shooter.getAverageVelocity() >= MIN_FORCE_FEED_SHOOTER_VELOCITY;
    }

    private String getCurrentFeedBlocker() {

        if (!lastShotPlan.aimReachable) {
            lastFeedBlocker = "aim unreachable";
        } else if (!turret.isSafe()) {
            lastFeedBlocker = "turret unsafe: " + turret.getFaultReason();
        } else if (!turret.isReady()) {
            lastFeedBlocker = "turret not ready";
        } else if (!hood.isReady()) {
            lastFeedBlocker = "hood not ready";
        } else if (shooter.getAverageVelocity() < MIN_FORCE_FEED_SHOOTER_VELOCITY) {
            lastFeedBlocker = "shooter below min";
        } else if (!shooter.isReady()) {
            lastFeedBlocker = "shooter below target";
        } else if (!shootCommand.isVisionReady() && !shootCommand.isFailsafeReady()) {
            lastFeedBlocker = "waiting vision/failsafe";
        } else {
            lastFeedBlocker = "unknown";
        }

        return lastFeedBlocker;
    }

    private ShotPlan calculateShotPlan(boolean shooting) {

        Pose pose =
                follower.getPose();

        ShotPlan plan =
                ballistics.calculateFromPose(
                        selectedGoal,
                        pose.getX(),
                        pose.getY(),
                        pose.getHeading(),
                        velocityXInchesPerSecond,
                        velocityYInchesPerSecond,
                        lastTarget,
                        lastBatteryVoltage,
                        shooting,
                        turret.getRobotRelativeDegrees()
                );

        if (shooting
                && (shooterVelocityBoost != 0.0
                || activeShotVelocityOffset != 0.0)) {
            plan.targetShooterVelocity =
                    Math.max(
                            0.0,
                            plan.targetShooterVelocity
                                    + shooterVelocityBoost
                                    + activeShotVelocityOffset
                    );
        }

        return plan;
    }

    private void finishShotSequence() {

        shotsRemaining = 0;
        activeShotVelocityOffset = 0.0;
        intakeEnabled = false;
        feeder.update(false, false, false, false);
        shooter.update(0);
        hood.holdParkPosition();
        shootCommand.resetReadiness();
        state = AutoShotState.IDLE;
        stateTimer.reset();
        lastReadyToFeed = false;
        lastFeedBlocker = "idle";
    }

    private void updatePoseVelocity() {

        Pose pose =
                follower.getPose();

        double dt =
                poseTimer.seconds();

        poseTimer.reset();

        if (lastPose != null && dt > 0.005 && dt < 0.25) {
            velocityXInchesPerSecond =
                    (pose.getX() - lastPose.getX()) / dt;
            velocityYInchesPerSecond =
                    (pose.getY() - lastPose.getY()) / dt;
        }

        lastPose =
                new Pose(
                        pose.getX(),
                        pose.getY(),
                        pose.getHeading()
                );
    }

    private void capturePoseSnapshot() {

        if (follower == null) {
            return;
        }

        Pose pose =
                follower.getPose();

        lastPose =
                new Pose(
                        pose.getX(),
                        pose.getY(),
                        pose.getHeading()
                );
    }

    private double readBatteryVoltage() {

        double bestVoltage =
                Double.POSITIVE_INFINITY;

        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage =
                    sensor.getVoltage();

            if (voltage > 0) {
                bestVoltage =
                        Math.min(bestVoltage, voltage);
            }
        }

        return Double.isInfinite(bestVoltage)
                ? Constants.Ballistics.NOMINAL_BATTERY_VOLTAGE
                : bestVoltage;
    }
}
