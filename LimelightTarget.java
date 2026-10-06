package org.firstinspires.ftc.teamcode.astrovos.util;

public class LimelightTarget {

    public final boolean valid;
    public final int tagId;
    public final double txDegrees;
    public final double tyDegrees;
    public final double distanceInches;
    public final boolean distanceValid;
    public final double totalLatencyMs;
    public final String reason;

    private LimelightTarget(
            boolean valid,
            int tagId,
            double txDegrees,
            double tyDegrees,
            double distanceInches,
            boolean distanceValid,
            double totalLatencyMs,
            String reason
    ) {

        this.valid = valid;
        this.tagId = tagId;
        this.txDegrees = txDegrees;
        this.tyDegrees = tyDegrees;
        this.distanceInches = distanceInches;
        this.distanceValid = distanceValid;
        this.totalLatencyMs = totalLatencyMs;
        this.reason = reason;
    }

    public static LimelightTarget valid(
            int tagId,
            double txDegrees,
            double tyDegrees,
            double distanceInches,
            boolean distanceValid,
            double totalLatencyMs
    ) {

        return new LimelightTarget(
                true,
                tagId,
                txDegrees,
                tyDegrees,
                distanceInches,
                distanceValid,
                totalLatencyMs,
                "selected tag"
        );
    }

    public static LimelightTarget inactive(String reason) {

        return new LimelightTarget(
                false,
                -1,
                0,
                0,
                0,
                false,
                0,
                reason
        );
    }
}
