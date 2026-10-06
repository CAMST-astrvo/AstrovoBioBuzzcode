package org.firstinspires.ftc.teamcode.AutoHelpers;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.astrovos.util.FieldCoordinates;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue 9 Far Shooter", group = "Autonomous")
@Configurable
public class blue9far extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private final AutoShotController shooter = new AutoShotController();
    private int pathState;
    private Paths paths;
    private boolean path1ShotStarted;
    private boolean path4ShotStarted;
    private boolean path7ShotStarted;

    @Override
    public void init() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(bluePose(82.400, 10.600, 90));
        paths = new Paths(follower);

        shooter.init(
                hardwareMap,
                follower,
                FieldCoordinates.AllianceGoal.BLUE
        );

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {

        follower.update();
        shooter.update();
        sendTelemetry();
    }

    @Override
    public void start() {

        setPathState(0);
    }

    @Override
    public void loop() {

        follower.update();
        shooter.update();
        autonomousPathUpdate();
        sendTelemetry();
    }

    @Override
    public void stop() {

        MatchData.finalAutoPose = follower.getPose();
        shooter.stop();
    }

    public void setPathState(int pState) {

        pathState = pState;
    }

    public void autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy() && finishPath1Shot()) {
                    shooter.setIntakeEnabled(true);
                    follower.followPath(paths.Path2);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    shooter.setIntakeEnabled(true);
                    follower.followPath(paths.Path3);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    shooter.setIntakeEnabled(true);
                    follower.followPath(paths.Path4);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy() && finishPath4Shot()) {
                    shooter.setIntakeEnabled(true);
                    follower.followPath(paths.Path5);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    shooter.setIntakeEnabled(true);
                    follower.followPath(paths.Path6);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    shooter.setIntakeEnabled(true);
                    follower.followPath(paths.Path7);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy() && finishPath7Shot()) {
                    setPathState(-1);
                }
                break;

            default:
                shooter.setIntakeEnabled(false);
                break;
        }
    }

    private boolean finishPath1Shot() {

        if (!path1ShotStarted) {
            shooter.fireShotsWithVelocityOffset(
                    1,
                    AutoShotController.FAR_FIRST_SHOT_VELOCITY_OFFSET
            );
            path1ShotStarted = true;
            return false;
        }

        return !shooter.isBusy();
    }

    private boolean finishPath4Shot() {

        if (!path4ShotStarted) {
            shooter.fireShots(1);
            path4ShotStarted = true;
            return false;
        }

        return !shooter.isBusy();
    }

    private boolean finishPath7Shot() {

        if (!path7ShotStarted) {
            shooter.fireShots(1);
            path7ShotStarted = true;
            return false;
        }

        return !shooter.isBusy();
    }

    private void sendTelemetry() {

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);

        telemetry.addData("Path State", pathState);
        telemetry.addData("Goal", FieldCoordinates.AllianceGoal.BLUE);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Auto Shot", "%s %s",
                shooter.getStateName(),
                shooter.getLastStatus());
        telemetry.addData("Feed blocker", shooter.getLastFeedBlocker());
        telemetry.addData("Feed gate", "mech %s vision %s failsafe %s %.2f",
                shooter.areMechanismsReady(),
                shooter.isVisionReady(),
                shooter.isFailsafeReady(),
                shooter.getFailsafeElapsedSeconds());
        telemetry.addData("Ready | turret/hood/shooter", "%s / %s / %s",
                shooter.isTurretReady(),
                shooter.isHoodReady(),
                shooter.isShooterReady());
        telemetry.addData("Turret safe/homed/fault", "%s / %s / %s",
                shooter.isTurretSafe(),
                shooter.isTurretHomed(),
                shooter.getTurretFaultReason());
        telemetry.addData("LL", "%s tx %.2f correction %s %.2f",
                shooter.getLastTarget().valid ? "valid" : shooter.getLastTarget().reason,
                shooter.getLastTarget().txDegrees,
                shooter.getLastShotPlan().usingLimelightCorrection ? "on" : "off",
                shooter.getLastShotPlan().filteredLimelightCorrectionDegrees);
        telemetry.addData("Shooter target/actual", "%.0f / %.0f",
                shooter.getTargetVelocity(),
                shooter.getVelocityReading());
        telemetry.addData("Pose", "%.1f, %.1f, %.1f",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private static Pose bluePose(
            double redX,
            double redY,
            double redHeadingDegrees
    ) {

        return new Pose(
                FieldCoordinates.FIELD_SIZE_INCHES - redX,
                redY,
                blueHeading(redHeadingDegrees)
        );
    }

    private static double blueHeading(double redHeadingDegrees) {

        double mirrored =
                180.0 - redHeadingDegrees;

        while (mirrored < 0) {
            mirrored += 360.0;
        }

        while (mirrored >= 360.0) {
            mirrored -= 360.0;
        }

        return Math.toRadians(mirrored);
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    bluePose(82.400, 10.600, 90),
                                    bluePose(81.629, 13.806, 74)
                            )
                    ).setLinearHeadingInterpolation(blueHeading(90), blueHeading(74))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    bluePose(81.629, 13.806, 74),
                                    bluePose(103.046, 35.406, 360)
                            )
                    ).setLinearHeadingInterpolation(blueHeading(74), blueHeading(360))
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    bluePose(103.046, 35.406, 360),
                                    bluePose(133.020, 35.312, 360)
                            )
                    ).setLinearHeadingInterpolation(blueHeading(360), blueHeading(360))
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    bluePose(133.020, 35.312, 360),
                                    bluePose(82.031, 14.064, 70)
                            )
                    ).setLinearHeadingInterpolation(blueHeading(360), blueHeading(70))
                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    bluePose(82.031, 14.064, 70),
                                    bluePose(100.354, 59.844, 360)
                            )
                    ).setLinearHeadingInterpolation(blueHeading(70), blueHeading(360))
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    bluePose(100.354, 59.844, 360),
                                    bluePose(126.435, 60.182, 360)
                            )
                    ).setLinearHeadingInterpolation(blueHeading(360), blueHeading(360))
                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    bluePose(126.435, 60.182, 360),
                                    bluePose(82.172, 14.004, 70)
                            )
                    ).setLinearHeadingInterpolation(blueHeading(360), blueHeading(70))
                    .build();
        }
    }
}
