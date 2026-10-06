package org.firstinspires.ftc.teamcode.astrovos.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.AutoHelpers.MatchData;
import org.firstinspires.ftc.teamcode.astrovos.util.Constants;
import org.firstinspires.ftc.teamcode.astrovos.util.FieldCoordinates;
import org.firstinspires.ftc.teamcode.astrovos.util.MathUtil;

import com.pedropathing.geometry.Pose;

public class LocalizationSubsystem {

    private final GoBildaPinpointDriver pinpoint;

    private Pose2D currentPose =
            new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);

    private double xInches;
    private double yInches;
    private double headingRadians;
    private double velocityXInchesPerSecond;
    private double velocityYInchesPerSecond;
    private double headingVelocityRadiansPerSecond;
    private String startPoseSource = "pinpoint reset";
    private double startPoseHeadingOffsetDegrees;

    public LocalizationSubsystem(HardwareMap hardwareMap) {

        pinpoint =
                hardwareMap.get(
                        GoBildaPinpointDriver.class,
                        Constants.Hardware.PINPOINT_NAME
                );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setOffsets(
                Constants.Localization.X_POD_OFFSET_MM,
                Constants.Localization.Y_POD_OFFSET_MM,
                DistanceUnit.MM
        );

        pinpoint.resetPosAndIMU();
    }

    public void update() {

        pinpoint.update();
        currentPose = pinpoint.getPosition();

        xInches =
                currentPose.getX(DistanceUnit.INCH);

        yInches =
                currentPose.getY(DistanceUnit.INCH);

        headingRadians =
                MathUtil.normalizeRadians(
                        currentPose.getHeading(AngleUnit.RADIANS)
                );

        velocityXInchesPerSecond =
                blend(
                        velocityXInchesPerSecond,
                        MathUtil.finiteOrZero(
                                pinpoint.getVelX(DistanceUnit.INCH)
                        ),
                        Constants.Localization.VELOCITY_FILTER_ALPHA
                );

        velocityYInchesPerSecond =
                blend(
                        velocityYInchesPerSecond,
                        MathUtil.finiteOrZero(
                                pinpoint.getVelY(DistanceUnit.INCH)
                        ),
                        Constants.Localization.VELOCITY_FILTER_ALPHA
                );

        headingVelocityRadiansPerSecond =
                MathUtil.finiteOrZero(
                        pinpoint.getHeadingVelocity(
                                UnnormalizedAngleUnit.RADIANS
                        )
                );
    }

    public void applyConfiguredStartPose(FieldCoordinates.AllianceGoal selectedGoal) {

        if (!Constants.Localization.RESET_PINPOINT_TO_START_POSE_ON_ALLIANCE_SELECT) {
            startPoseSource = "pinpoint existing pose";
            startPoseHeadingOffsetDegrees = 0.0;
            return;
        }

        if (Constants.Localization.USE_LAST_AUTO_POSE_IN_TELEOP
                && MatchData.finalAutoPose != null) {
            Pose autoPose =
                    MatchData.finalAutoPose;

            double headingOffsetDegrees =
                    selectedGoal == FieldCoordinates.AllianceGoal.BLUE
                            ? Constants.Localization.BLUE_LAST_AUTO_HEADING_OFFSET_DEGREES
                            : Constants.Localization.RED_LAST_AUTO_HEADING_OFFSET_DEGREES;

            currentPose =
                    new Pose2D(
                            DistanceUnit.INCH,
                            autoPose.getX(),
                            autoPose.getY(),
                            AngleUnit.RADIANS,
                            autoPose.getHeading()
                                    + Math.toRadians(headingOffsetDegrees)
                    );

            startPoseSource = "last auto pose";
            startPoseHeadingOffsetDegrees = headingOffsetDegrees;
            pinpoint.setPosition(currentPose);
            return;
        }

        double startX =
                selectedGoal == FieldCoordinates.AllianceGoal.BLUE
                        ? Constants.Localization.BLUE_START_X_INCHES
                        : Constants.Localization.RED_START_X_INCHES;

        double startY =
                selectedGoal == FieldCoordinates.AllianceGoal.BLUE
                        ? Constants.Localization.BLUE_START_Y_INCHES
                        : Constants.Localization.RED_START_Y_INCHES;

        double startHeadingDegrees =
                selectedGoal == FieldCoordinates.AllianceGoal.BLUE
                        ? Constants.Localization.BLUE_START_HEADING_DEGREES
                        : Constants.Localization.RED_START_HEADING_DEGREES;

        currentPose =
                new Pose2D(
                        DistanceUnit.INCH,
                        startX,
                        startY,
                        AngleUnit.DEGREES,
                        startHeadingDegrees
                );

        pinpoint.setPosition(currentPose);
        startPoseSource = "constants " + selectedGoal;
        startPoseHeadingOffsetDegrees = 0.0;
    }

    public String getStartPoseSource() {

        return startPoseSource;
    }

    public double getStartPoseHeadingOffsetDegrees() {

        return startPoseHeadingOffsetDegrees;
    }

    public GoBildaPinpointDriver.DeviceStatus getDeviceStatus() {

        return pinpoint.getDeviceStatus();
    }

    public double getXInches() {

        return xInches;
    }

    public double getYInches() {

        return yInches;
    }

    public double getHeadingRadians() {

        return headingRadians;
    }

    public double getVelocityXInchesPerSecond() {

        return velocityXInchesPerSecond;
    }

    public double getVelocityYInchesPerSecond() {

        return velocityYInchesPerSecond;
    }

    public double getHeadingVelocityRadiansPerSecond() {

        return headingVelocityRadiansPerSecond;
    }

    public double getSpeedInchesPerSecond() {

        return Math.hypot(
                velocityXInchesPerSecond,
                velocityYInchesPerSecond
        );
    }

    private double blend(
            double oldValue,
            double newValue,
            double alpha
    ) {

        return (oldValue * (1.0 - alpha))
                + (newValue * alpha);
    }
}
