package org.firstinspires.ftc.teamcode.astrovos.util;

public class AllianceManager {

    private FieldCoordinates.AllianceGoal selectedGoal =
            FieldCoordinates.AllianceGoal.BLUE;

    private boolean previousDpadLeft;
    private boolean previousDpadRight;

    public void updateFromDpad(
            boolean dpadLeft,
            boolean dpadRight
    ) {

        if (dpadLeft && !previousDpadLeft) {
            selectedGoal =
                    FieldCoordinates.AllianceGoal.BLUE;
        }

        if (dpadRight && !previousDpadRight) {
            selectedGoal =
                    FieldCoordinates.AllianceGoal.RED;
        }

        previousDpadLeft = dpadLeft;
        previousDpadRight = dpadRight;
    }

    public FieldCoordinates.AllianceGoal getSelectedGoal() {

        return selectedGoal;
    }

    public double getDriverFieldHeadingRadians(double robotFieldHeadingRadians) {

        return selectedGoal == FieldCoordinates.AllianceGoal.BLUE
                ? robotFieldHeadingRadians + Math.PI
                : robotFieldHeadingRadians;
    }
}
