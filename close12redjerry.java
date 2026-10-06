package org.firstinspires.ftc.teamcode.AutoHelpers;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.astrovos.util.FieldCoordinates;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Close 12 Red Jerry Shooter", group = "Autonomous")
public class close12redjerry extends OpMode {

    private Follower follower;
    private final AutoShotController shooter = new AutoShotController();
    private final boolean[] shotStarted = new boolean[4];
    private int pathState;
    private Paths paths;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(113.000, 134.000, Math.toRadians(90)));
        paths = new Paths(follower);

        shooter.init(
                hardwareMap,
                follower,
                FieldCoordinates.AllianceGoal.RED
        );
        shooter.useCloseAutoShooterSpeed();
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

    private void setPathState(int pathState) {

        this.pathState = pathState;
    }

    private void autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy() && finishShotIfNeeded(0)) {
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
                if (!follower.isBusy() && finishShotIfNeeded(1)) {
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
                if (!follower.isBusy() && finishShotIfNeeded(2)) {
                    shooter.setIntakeEnabled(true);
                    follower.followPath(paths.Path8);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    shooter.setIntakeEnabled(true);
                    follower.followPath(paths.Path9);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    shooter.setIntakeEnabled(true);
                    follower.followPath(paths.Path10);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy() && finishShotIfNeeded(3)) {
                    setPathState(-1);
                }
                break;

            default:
                shooter.setIntakeEnabled(false);
                break;
        }
    }

    private boolean finishShotIfNeeded(int shotIndex) {

        if (!shotStarted[shotIndex]) {
            shooter.fireShots(1);
            shotStarted[shotIndex] = true;
            return false;
        }

        return !shooter.isBusy();
    }

    private void sendTelemetry() {

        telemetry.addData("Path State", pathState);
        telemetry.addData("Goal", FieldCoordinates.AllianceGoal.RED);
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

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(113.000, 134.000),
                                    new Pose(99.693, 109.573)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(99.693, 109.573),
                                    new Pose(86.385, 85.145)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(53))
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.385, 85.145),
                                    new Pose(123.106, 84.789)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(360))
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123.106, 84.789),
                                    new Pose(86.565, 85.024)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(53))
                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.565, 85.024),
                                    new Pose(98.398, 60.817)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(53), Math.toRadians(360))
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(98.398, 60.817),
                                    new Pose(125.505, 60.602)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(360))
                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.505, 60.602),
                                    new Pose(87.319, 85.327)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(53))
                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.319, 85.327),
                                    new Pose(98.152, 36.128)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(53), Math.toRadians(360))
                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(98.152, 36.128),
                                    new Pose(129.831, 36.006)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(360))
                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(129.831, 36.006),
                                    new Pose(88.363, 85.259)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(53))
                    .build();
        }
    }
}
