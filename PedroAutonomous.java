package org.firstinspires.ftc.teamcode.AutoHelpers;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.astrovos.util.FieldCoordinates;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Autonomous Shooter", group = "Autonomous")
public class PedroAutonomous extends OpMode {

    private Follower follower;
    private AutoShotController shooter = new AutoShotController();
    private int pathState;
    private final ElapsedTime pathTimer = new ElapsedTime();
    private final boolean[] shotStarted = new boolean[8];
    private Paths paths;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(32, 136, Math.toRadians(90)));
        paths = new Paths(follower);

        shooter.init(
                hardwareMap,
                follower,
                FieldCoordinates.AllianceGoal.BLUE
        );
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
        pathTimer.reset();
    }

    public void autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                follower.followPath(paths.StarterPosition);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy() && finishShotIfNeeded(0)) {
                    shooter.setIntakeEnabled(true);
                    follower.followPath(paths.CollectArtifacts1, 0.7, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy() && finishShotIfNeeded(1)) {
                    shooter.setIntakeEnabled(true);
                    follower.followPath(paths.Path3);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy() && finishShotIfNeeded(2)) {
                    shooter.setIntakeEnabled(true);
                    follower.followPath(paths.Path4);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy() && finishShotIfNeeded(3)) {
                    shooter.setIntakeEnabled(true);
                    follower.followPath(paths.Path5);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy() && finishShotIfNeeded(4)) {
                    shooter.setIntakeEnabled(true);
                    follower.followPath(paths.Path6);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy() && finishShotIfNeeded(5)) {
                    shooter.setIntakeEnabled(true);
                    follower.followPath(paths.Path7);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy() && finishShotIfNeeded(6)) {
                    shooter.setIntakeEnabled(true);
                    follower.followPath(paths.Path8);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy() && finishShotIfNeeded(7)) {
                    setPathState(-1);
                }
                break;

            default:
                shooter.setIntakeEnabled(false);
                break;
        }
    }

    private boolean finishShotIfNeeded(int stopIndex) {

        if (!shotStarted[stopIndex]) {
            shooter.fireShots(1);
            shotStarted[stopIndex] = true;
            return false;
        }

        return !shooter.isBusy();
    }

    private void sendTelemetry() {

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

    public static class Paths {
        public PathChain StarterPosition;
        public PathChain CollectArtifacts1;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;

        public Paths(Follower follower) {

            StarterPosition = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(32, 136),
                                    new Pose(42, 100)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(133))
                    .build();

            CollectArtifacts1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42, 100),
                                    new Pose(43, 84)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(43, 84),
                                    new Pose(16, 83)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(133))
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16, 83),
                                    new Pose(40, 102)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(180))
                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(40, 102),
                                    new Pose(45, 60)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(45, 60),
                                    new Pose(12, 59)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(133))
                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12, 59),
                                    new Pose(45, 97)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(154))
                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14.161, 60.583),
                                    new Pose(58.936, 84.572)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(133))
                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14.161, 60.583),
                                    new Pose(58.936, 84.572)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(133))
                    .build();
        }
    }
}
