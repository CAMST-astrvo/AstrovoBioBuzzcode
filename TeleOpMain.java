package org.firstinspires.ftc.teamcode.astrovos;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.astrovos.commands.AimCommand;
import org.firstinspires.ftc.teamcode.astrovos.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.astrovos.commands.TrackGoalCommand;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.BallisticsSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.astrovos.util.AllianceManager;
import org.firstinspires.ftc.teamcode.astrovos.util.Constants;
import org.firstinspires.ftc.teamcode.astrovos.util.FieldCoordinates;
import org.firstinspires.ftc.teamcode.astrovos.util.LimelightTarget;
import org.firstinspires.ftc.teamcode.astrovos.util.ShotPlan;

import java.util.List;

@TeleOp(name = "Astrovos Goal Lock", group = "Astrovos")
public class TeleOpMain extends LinearOpMode {

    private AllianceManager allianceManager;
    private LocalizationSubsystem localization;
    private DriveSubsystem drive;
    private TurretSubsystem turret;
    private ShooterSubsystem shooter;
    private HoodSubsystem hood;
    private FeederSubsystem feeder;
    private VisionSubsystem vision;
    private BallisticsSubsystem ballistics;
    private TrackGoalCommand trackGoalCommand;
    private AimCommand aimCommand;
    private ShootCommand shootCommand;

    @Override
    public void runOpMode() {

        configureBulkCaching();
        telemetry.setMsTransmissionInterval(40);

        allianceManager = new AllianceManager();
        localization = new LocalizationSubsystem(hardwareMap);
        drive = new DriveSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        hood = new HoodSubsystem(hardwareMap);
        feeder = new FeederSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        ballistics = new BallisticsSubsystem();

        trackGoalCommand =
                new TrackGoalCommand(
                        localization,
                        turret,
                        ballistics
                );

        aimCommand =
                new AimCommand(
                        localization,
                        turret,
                        shooter,
                        hood,
                        ballistics
                );

        shootCommand =
                new ShootCommand(
                        turret,
                        shooter,
                        hood,
                        feeder,
                        vision
                );

        initLoop();

        if (isStopRequested()) {
            stopAll();
            return;
        }

        localization.applyConfiguredStartPose(
                allianceManager.getSelectedGoal()
        );

        while (opModeIsActive()) {
            DriverControls controls =
                    readControls();

            localization.update();
            vision.update(
                    controls.shootHeld,
                    allianceManager.getSelectedGoal()
            );

            double batteryVoltage =
                    getBatteryVoltage();

            if (controls.turretHomeHeld) {
                turret.requestHomeRecovery();
            } else if (turret.isHomeRecoveryActive()) {
                turret.cancelHomeRecovery();
            }

            boolean turretRecoveryActive =
                    turret.isHomeRecoveryActive();

            ShotPlan shotPlan;

            if (turretRecoveryActive) {
                turret.updateHomeRecovery(controls.turretHomeHeld);
                shotPlan =
                        ballistics.calculate(
                                allianceManager.getSelectedGoal(),
                                localization,
                                LimelightTarget.inactive("turret home recovery"),
                                batteryVoltage,
                                false,
                                turret.getRobotRelativeDegrees()
                        );
                shooter.update(0);
                hood.holdParkPosition();
            } else if (controls.shootHeld) {
                shotPlan =
                        aimCommand.execute(
                                allianceManager.getSelectedGoal(),
                                vision.getLastTarget(),
                                batteryVoltage,
                                true
                        );
            } else {
                shotPlan =
                        trackGoalCommand.execute(
                                allianceManager.getSelectedGoal(),
                                batteryVoltage
                        );
                shooter.update(0);
                hood.holdParkPosition();
            }

            boolean readyToFeed =
                    shootCommand.execute(
                            controls.shootHeld && !turretRecoveryActive,
                            shotPlan,
                            controls.manualIntake
                                    && !controls.shootHeld
                                    && !turretRecoveryActive,
                            controls.reverseIntake
                                    && !controls.shootHeld
                                    && !turretRecoveryActive,
                            controls.kickerHeld && !turretRecoveryActive
                    );

            drive.drive(
                    controls.driveX,
                    controls.driveY,
                    controls.driveTurn,
                    controls.precisionMode,
                    allianceManager.getDriverFieldHeadingRadians(
                            localization.getHeadingRadians()
                    )
            );

            sendTelemetry(
                    controls,
                    shotPlan,
                    readyToFeed,
                    batteryVoltage
            );
        }

        stopAll();
    }

    private void initLoop() {

        boolean previousInitTurretHomeHeld =
                false;

        while (!isStarted() && !isStopRequested()) {
            allianceManager.updateFromDpad(
                    gamepad1.dpad_left,
                    gamepad1.dpad_right
            );

            localization.update();

            ShotPlan initPlan =
                    ballistics.calculate(
                            allianceManager.getSelectedGoal(),
                            localization,
                            LimelightTarget.inactive("init preview only"),
                            getBatteryVoltage(),
                            false,
                            turret.getRobotRelativeDegrees()
                    );

            boolean initTurretHomeHeld =
                    gamepad1.a || gamepad1.y;

            if (initTurretHomeHeld) {
                if (!previousInitTurretHomeHeld) {
                    turret.cancelHomeRecovery();
                }

                turret.requestHomeRecovery();
                turret.updateHomeRecovery(true);
            } else {
                if (turret.isHomeRecoveryActive()) {
                    turret.cancelHomeRecovery();
                }

                turret.updateHoming(false);
            }

            previousInitTurretHomeHeld =
                    initTurretHomeHeld;

            shooter.update(0);
            hood.holdParkPosition();
            feeder.update(false, false, false, false);

            telemetry.addLine("Astrovos Goal Lock initialized");
            telemetry.addData("Init select", "dpad left blue | dpad right red");
            telemetry.addData("Selected goal", allianceManager.getSelectedGoal());
            telemetry.addData("Goal point", "%.1f, %.1f tag %d",
                    allianceManager.getSelectedGoal().xInches,
                    allianceManager.getSelectedGoal().yInches,
                    allianceManager.getSelectedGoal().aprilTagId);
            telemetry.addData("Pose in", "%.1f, %.1f, %.1f",
                    localization.getXInches(),
                    localization.getYInches(),
                    Math.toDegrees(localization.getHeadingRadians()));
            telemetry.addData("Turret", "%.1f target %.1f safe %s",
                    turret.getRobotRelativeDegrees(),
                    initPlan.turretTargetRobotDegrees,
                    turret.isSafe());
            telemetry.addData("Turret travel from switch", "%.1f",
                    turret.getTravelDegrees());
            telemetry.addData("Turret homed/homing", "%s / %s",
                    turret.isHomed(),
                    turret.isHoming());
            telemetry.addData("Turret touch raw/latched/fault", "%s / %s / %s",
                    turret.isRawHomeSensorPressed(),
                    turret.isHomeSwitchPressed(),
                    turret.isFaulted());
            telemetry.addData("Turret fault reason", turret.getFaultReason());
            telemetry.addData("Turret motor power", "%.2f",
                    turret.getLastMotorPower());
            telemetry.addData("Pinpoint", localization.getDeviceStatus());
            telemetry.addData("Turret homes toward touch sensor at", "%.1f deg",
                    Constants.Turret.MIN_ROBOT_RELATIVE_DEGREES);
            telemetry.addLine("Hold A or Y to home turret in init; release to stop/retry.");
            telemetry.update();

            idle();
        }
    }

    private DriverControls readControls() {

        DriverControls controls =
                new DriverControls();

        controls.driveX = gamepad1.left_stick_x;
        controls.driveY = -gamepad1.left_stick_y;
        controls.driveTurn = gamepad1.right_stick_x;
        controls.precisionMode = gamepad1.left_trigger > 0.35;
        controls.shootHeld = gamepad1.left_bumper;
        controls.manualIntake = gamepad1.right_bumper;
        controls.reverseIntake = gamepad1.right_trigger > 0.35;
        controls.kickerHeld = gamepad1.a;
        controls.turretHomeHeld = gamepad1.y;

        return controls;
    }

    private void configureBulkCaching() {

        List<LynxModule> hubs =
                hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    private double getBatteryVoltage() {

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

    private void sendTelemetry(
            DriverControls controls,
            ShotPlan shotPlan,
            boolean readyToFeed,
            double batteryVoltage
    ) {

        FieldCoordinates.AllianceGoal goal =
                allianceManager.getSelectedGoal();

        LimelightTarget target =
                vision.getLastTarget();

        telemetry.addData("Goal", "%s %.1f, %.1f tag %d",
                goal,
                goal.xInches,
                goal.yInches,
                goal.aprilTagId);
        telemetry.addData("Field drive", "%s perspective", goal);
        telemetry.addData("Shoot held", controls.shootHeld);
        telemetry.addData("Turret home button/recovery", "%s / %s",
                controls.turretHomeHeld,
                turret.isHomeRecoveryActive());
        telemetry.addData("Ready to feed", readyToFeed);
        telemetry.addData("Ready | shoot/aim", "%s / %s",
                controls.shootHeld,
                shotPlan.aimReachable);
        telemetry.addData("Ready | turret/hood/shooter", "%s / %s / %s",
                turret.isReady(),
                hood.isReady(),
                shooter.isReady());
        telemetry.addData("Ready | vision/failsafe", "%s / %s",
                shootCommand.isVisionReady(),
                shootCommand.isFailsafeReady());
        telemetry.addData("Feed gate", "mech %s vision %s failsafe %s %.2f/%.2f",
                shootCommand.areMechanismsReady(),
                shootCommand.isVisionReady(),
                shootCommand.isFailsafeReady(),
                shootCommand.getFailsafeElapsedSeconds(),
                Constants.Feeder.VISION_READY_FAILSAFE_SECONDS);
        telemetry.addData("Battery V", "%.2f", batteryVoltage);
        telemetry.addData("Pose in", "%.1f, %.1f, %.1f",
                localization.getXInches(),
                localization.getYInches(),
                Math.toDegrees(localization.getHeadingRadians()));
        telemetry.addData("Pose source", "%s heading offset %.1f deg",
                localization.getStartPoseSource(),
                localization.getStartPoseHeadingOffsetDegrees());
        telemetry.addData("Velocity in/s", "%.1f, %.1f speed %.1f",
                localization.getVelocityXInchesPerSecond(),
                localization.getVelocityYInchesPerSecond(),
                localization.getSpeedInchesPerSecond());
        telemetry.addData("Distance in", "%.1f (%s)",
                shotPlan.distanceInches,
                shotPlan.usingLimelightDistance ? "limelight" : "odometry");
        telemetry.addData("Odo distance in", "%.1f", shotPlan.odometryDistanceInches);
        telemetry.addData("Lead time s", "%.2f", shotPlan.flightTimeSeconds);
        telemetry.addData("Turret", "%.1f target %.1f error %.1f",
                turret.getRobotRelativeDegrees(),
                turret.getLastTargetRobotDegrees(),
                turret.getLastErrorDegrees());
        telemetry.addData("Turret travel from switch", "%.1f",
                turret.getTravelDegrees());
        telemetry.addData("Turret ready/safe", "%s / %s",
                turret.isReady(),
                turret.isSafe());
        telemetry.addData("Turret homed/homing/rehome/touch raw/latched/fault",
                "%s / %s / %s / %s / %s / %s",
                turret.isHomed(),
                turret.isHoming(),
                turret.isHomeRecoveryActive(),
                turret.isRawHomeSensorPressed(),
                turret.isHomeSwitchPressed(),
                turret.isFaulted());
        telemetry.addData("Turret fault reason", turret.getFaultReason());
        telemetry.addData("Turret motor power", "%.2f",
                turret.getLastMotorPower());
        telemetry.addData("Turret clamped", turret.wasLastTargetClamped());
        telemetry.addData("LL", "%s reason %s",
                target.valid ? "valid" : "not valid",
                target.reason);
        telemetry.addData("LL hw", "available %s connected %s statusErr %s",
                vision.isAvailable(),
                vision.isConnected(),
                vision.getLastStatusError());
        telemetry.addData("LL sample/result", "sampled %s present %s valid %s readErr %s",
                vision.wasLastSampleAttempted(),
                vision.wasLastResultPresent(),
                vision.wasLastResultValid(),
                vision.getLastReadError());
        telemetry.addData("LL pipeline", "%d %s",
                vision.getLastPipelineIndex(),
                vision.getLastPipelineType());
        telemetry.addData("LL fiducials", "%d ids %s",
                vision.getLastFiducialCount(),
                vision.getLastFiducialIds());
        telemetry.addData("LL latency ms", "stale %.1f cap %.1f target %.1f total %.1f",
                vision.getLastStalenessMs(),
                vision.getLastCaptureLatencyMs(),
                vision.getLastTargetingLatencyMs(),
                vision.getLastTotalLatencyMs());
        telemetry.addData("LL tag/tx/ty", "%d %.2f %.2f",
                target.tagId,
                target.txDegrees,
                target.tyDegrees);
        telemetry.addData("LL distance in", target.distanceValid ? "%.1f" : "not valid",
                target.distanceInches);
        telemetry.addData("LL ready", vision.isReadyToShoot());
        telemetry.addData("LL correction deg", "%s raw %.2f filt %.2f",
                shotPlan.usingLimelightCorrection ? "on" : "off",
                shotPlan.rawLimelightCorrectionDegrees,
                shotPlan.filteredLimelightCorrectionDegrees);
        telemetry.addData("Shooter target/actual +shot1 and shot2 speed", "%.0f / %.0f",
                shooter.getTargetVelocity(),
                (shooter.getAverageVelocity()),
        telemetry.addData("Shooter plan", "base %.0f scale %.2f move %.0f",
                shotPlan.baseShooterVelocity,
                shotPlan.voltageScale,
                shotPlan.movingShotVelocityCompensation),
        telemetry.addData("Shooter motor type", "%s | %s",
                shooter.getShot1MotorTypeName(),
                shooter.getShot2MotorTypeName()),
        telemetry.addData("Shooter max tps", "%.0f | %.0f",
                shooter.getShot1ConfiguredMaxTicksPerSecond(),
                shooter.getShot2ConfiguredMaxTicksPerSecond()),
        telemetry.addData("Shooter tpr/rpm/fraction", "%.0f %.0f %.2f | %.0f %.0f %.2f",
                shooter.getShot1TicksPerRev(),
                shooter.getShot1MaxRpm(),
                shooter.getShot1AchievableMaxRpmFraction(),
                shooter.getShot2TicksPerRev(),
                shooter.getShot2MaxRpm(),
                shooter.getShot2AchievableMaxRpmFraction()),
        telemetry.addData("Shooter power", "%.2f | %.2f diag %s",
                shooter.getShot1Power(),
                shooter.getShot2Power(),
                shooter.isOpenLoopDiagnosticActive()));
        PIDFCoefficients shooterPidf =
                shooter.getShot1VelocityPidf();
        telemetry.addData("Shooter PIDF", "p %.1f i %.4f d %.4f f %.1f",
                shooterPidf.p,
                shooterPidf.i,
                shooterPidf.d,
                shooterPidf.f);
        telemetry.addData("Shooter ready", shooter.isReady());
        telemetry.addData("Hood pos", "%.3f ready %s",
                hood.getTargetPosition(),
                hood.isReady());
        telemetry.addData("Stopper", feeder.isStopperOpen() ? "open" : "closed");
        telemetry.addData("Intake", feeder.getIntakeMode());
        telemetry.addData("Kicker", feeder.isKickerFiring() ? "fire" : "rest");
        telemetry.addData("Controls", "L1 hold shoot | Y hold turret home | R1 intake | RT reverse | A kicker");
        telemetry.addData("shot1 speed",shooter.getshot1velocity());
        telemetry.addData("shot2 speed" , shooter.getshot2velocity());
        telemetry.update();
    }

    private void stopAll() {

        if (drive != null) {
            drive.stop();
        }

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

    private static final class DriverControls {

        double driveX;
        double driveY;
        double driveTurn;
        boolean precisionMode;
        boolean shootHeld;
        boolean manualIntake;
        boolean reverseIntake;
        boolean kickerHeld;
        boolean turretHomeHeld;
    }
}
