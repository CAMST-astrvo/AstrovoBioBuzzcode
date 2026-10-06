package org.firstinspires.ftc.teamcode.astrovos.util;

public final class FieldCoordinates {

    public static final double FIELD_SIZE_INCHES = 144.0;

    private FieldCoordinates() {
    }

    public enum AllianceGoal {
        BLUE(13.0, 138.0, 20),
        RED(131.0, 131.0, 24);

        public final double xInches;
        public final double yInches;
        public final int aprilTagId;

        AllianceGoal(
                double xInches,
                double yInches,
                int aprilTagId
        ) {

            this.xInches = xInches;
            this.yInches = yInches;
            this.aprilTagId = aprilTagId;
        }
    }
}
