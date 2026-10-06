package org.firstinspires.ftc.teamcode.AstroAuto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AutoHelpers.AutoShotController;
import org.firstinspires.ftc.teamcode.AutoHelpers.MatchData;
import org.firstinspires.ftc.teamcode.astrovos.util.FieldCoordinates;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable

@Autonomous(name="Gate Pickup", group="Pedro")
public class BiggerRed extends OpMode {

    double Pickup1 = 91.525096;
    double Pickup2 = 58;
    double Pickup3 = 43.5;
    public static double Depth = 117.9;
    public static double angle = 22;
    public static double Hight = 61.6;
    public static double seconds = 3.7;
    public static double Score_Angle = -134;



    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private AutoShotController shooter = new AutoShotController();

    private boolean shot1Triggered = false;
    private boolean shot2Triggered = false;
    private boolean shot3Triggered = false;
    private boolean shot4Triggered = false;
    private boolean shot5Triggered = false;
    private boolean shot6Triggered = false;
    private boolean shot7Triggered = false;

    private int pathState;

    private final Pose startPose = new Pose(125.54, 128.65473, Math.toRadians(-135));
    private final Pose scorePose = new Pose(92.270326, 98.6191979, Math.toRadians(Score_Angle));
    private final Pose prepickupPose = new Pose(84.335907, Pickup1, Math.toRadians(0));
    private final Pose pickupPose = new Pose(119, 87, Math.toRadians(0));
    private final Pose prepickupPose1 = new Pose(84.335907, 66, Math.toRadians(0));
    private final Pose pickupPose1 = new Pose(124, Pickup2, Math.toRadians(0));
    private final Pose gate = new Pose(Depth, Hight, Math.toRadians(angle));
    private final Pose pregate = new Pose(89, Pickup2, Math.toRadians(0));
    private final Pose prepickupPose2 = new Pose(84.335907, Pickup3, Math.toRadians(0));
    private final Pose pickupPose2 = new Pose(124, Pickup3, Math.toRadians(0));


    private final Pose parkPose = new Pose(86.22393, 127.84942, Math.toRadians(90));

    private Path scorePreload;
    private PathChain Park, PrePickup, Preload, scorePickup,PrePickup1, Preload1, scorePickup1,PrePickup2, Preload2,scorePickup2,PreGate1,ScoreGate1 ;

    public void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        PrePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, prepickupPose1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prepickupPose.getHeading())
                .build();

        Preload1 = follower.pathBuilder()
                .addPath(new BezierLine(prepickupPose1, pickupPose1))
                .setLinearHeadingInterpolation(prepickupPose.getHeading(), pickupPose.getHeading())
                .setVelocityConstraint(0.4)
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(prepickupPose1, scorePose))
                .setLinearHeadingInterpolation(pickupPose1.getHeading(), scorePose.getHeading())
                .build();

        PreGate1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, gate))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gate.getHeading())
                .build();

        ScoreGate1 = follower.pathBuilder()
                .addPath(new BezierLine(gate, scorePose))
                .setLinearHeadingInterpolation(gate.getHeading(), scorePose.getHeading())
                .build();

        PrePickup = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, prepickupPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prepickupPose.getHeading())
                .build();

        Preload = follower.pathBuilder()
                .addPath(new BezierLine(prepickupPose, pickupPose))
                .setLinearHeadingInterpolation(prepickupPose.getHeading(), pickupPose.getHeading())
                .setVelocityConstraint(0.4)
                .build();

        scorePickup = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, scorePose))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), scorePose.getHeading())
                .build();



        //PrePickup2 = follower.pathBuilder()
        //      .addPath(new BezierLine(scorePose, prepickupPose2))
        //    .setLinearHeadingInterpolation(scorePose.getHeading(), prepickupPose.getHeading())
        //   .build();

        // Preload2 = follower.pathBuilder()
        //      .addPath(new BezierLine(prepickupPose2, pickupPose2))
        //      .setLinearHeadingInterpolation(prepickupPose.getHeading(), pickupPose.getHeading())
        //       .setVelocityConstraint(0.4)
        //       .build();

        // scorePickup2 = follower.pathBuilder()
        //      .addPath(new BezierLine(pickupPose2, scorePose))
        //   .setLinearHeadingInterpolation(pickupPose.getHeading(), scorePose.getHeading())
        //   .build();

        Park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {

        switch (pathState) {

            // ===== PRELOAD SHOT =====

            case 0:
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {

                    if (!shot1Triggered) {
                        shooter.fireShots(1);
                        shot1Triggered = true;
                    }
                    else if (!shooter.isBusy()) {
                        follower.followPath(PrePickup1, true);
                        setPathState(2);
                    }
                }
                break;

            // ===== PICKUP 1 =====

            case 2:
                if (!follower.isBusy()) {
                    shooter.setIntakeEnabled(true);
                    follower.followPath(Preload1, true);   // go to pickup
                    setPathState(4);
                }
                break;


            case 4:
                if (!follower.isBusy()) {
                    //bench.setIntakeSpeed(0);
                    follower.followPath(scorePickup1, true);  // RETURN TO SCORE
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {

                    if (!shot2Triggered) {
                        shooter.fireShots(1);
                        shot2Triggered = true;
                    }
                    else if (!shooter.isBusy()) {
                        shooter.setIntakeEnabled(true);
                        follower.followPath(PreGate1, true);
                        setPathState(6);
                    }
                }
                break;

            // ===== PICKUP 2 =====

            case 6:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() >= seconds) {
                        follower.followPath(ScoreGate1, true);
                        setPathState(7);
                    }
                }
                break;

            case 7:
                if (!follower.isBusy() && follower.atParametricEnd()) {

                    if (!shot3Triggered) {
                        shooter.fireShots(1);
                        shot3Triggered = true;
                    }
                    else if (!shooter.isBusy()) {
                        follower.followPath(PreGate1, true);
                        shooter.setIntakeEnabled(true);
                        setPathState(8);
                    }
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() >= seconds) {
                        follower.followPath(ScoreGate1, true);
                        setPathState(9);
                    }
                }
                break;

            // ===== PICKUP 3 =====

            case 9:
                if (!follower.isBusy() && follower.atParametricEnd()) {

                    if (!shot4Triggered) {
                        shooter.fireShots(1);
                        shot4Triggered = true;
                    }
                    else if (!shooter.isBusy()) {
                        follower.followPath(PreGate1, true);
                        shooter.setIntakeEnabled(true);
                        setPathState(10);
                    }
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() >= seconds) {
                        follower.followPath(ScoreGate1, true);
                        setPathState(11);
                    }
                }
                break;

            case 11:
                if (!follower.isBusy() && follower.atParametricEnd()) {

                    if (!shot5Triggered) {
                        shooter.fireShots(1);
                        shot5Triggered = true;
                    }
                    else if (!shooter.isBusy()) {
                        follower.followPath(PreGate1, true);
                        shooter.setIntakeEnabled(true);
                        setPathState(12);
                    }
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() >= seconds) {
                        follower.followPath(ScoreGate1, true);
                        setPathState(13);
                    }
                }
                break;

            case 13:
                if (!follower.isBusy() && follower.atParametricEnd()) {

                    if (!shot6Triggered) {
                        shooter.fireShots(1);
                        shot6Triggered = true;
                    }
                    else if (!shooter.isBusy()) {
                        follower.followPath(PrePickup, true);
                        setPathState(14);
                    }
                }
                break;

            // ===== PICKUP 2 =====

            case 14:
                if (!follower.isBusy()) {
                    shooter.setIntakeEnabled(true);
                    follower.followPath(Preload, true);
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    // bench.setIntakeSpeed(0);
                    follower.followPath(scorePickup, true);   // RETURN
                    setPathState(16);
                }
                break;

            case 16:
                if (!follower.isBusy() && follower.atParametricEnd()) {

                    if (!shot7Triggered) {
                        shooter.fireShots(1);
                        shot7Triggered = true;
                    }
                    else if (!shooter.isBusy()) {
                        follower.followPath(Park, true);
                        setPathState(17);
                    }
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        follower.update();
        shooter.update();
        autonomousPathUpdate();


        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Shooter Busy", shooter.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Shooter Velocity", shooter.getVelocityReading());
        telemetry.addData("Shooter Target", shooter.getTargetVelocity());
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
        telemetry.update();
    }

    @Override
    public void init_loop() {

        follower.update();
        shooter.update();

        telemetry.addData("Auto", "Red turret/shooter readying");
        telemetry.addData("Auto Shot", "%s %s",
                shooter.getStateName(),
                shooter.getLastStatus());
        telemetry.addData("Feed blocker", shooter.getLastFeedBlocker());
        telemetry.addData("Ready | turret/hood/shooter", "%s / %s / %s",
                shooter.isTurretReady(),
                shooter.isHoodReady(),
                shooter.isShooterReady());
        telemetry.addData("LL", "%s tx %.2f",
                shooter.getLastTarget().valid ? "valid" : shooter.getLastTarget().reason,
                shooter.getLastTarget().txDegrees);
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        shooter.init(
                hardwareMap,
                follower,
                FieldCoordinates.AllianceGoal.RED
        );
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
        MatchData.finalAutoPose = follower.getPose();
        shooter.stop();
    }
}
