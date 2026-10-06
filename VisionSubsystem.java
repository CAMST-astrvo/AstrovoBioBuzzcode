package org.firstinspires.ftc.teamcode.astrovos.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.astrovos.util.Constants;
import org.firstinspires.ftc.teamcode.astrovos.util.FieldCoordinates;
import org.firstinspires.ftc.teamcode.astrovos.util.LimelightTarget;
import org.firstinspires.ftc.teamcode.astrovos.util.MathUtil;

import java.util.List;

public class VisionSubsystem {

    private Limelight3A limelight;
    private boolean available;
    private boolean connected;
    private boolean lastSampleAttempted;
    private boolean lastResultPresent;
    private boolean lastResultValid;
    private int lastPipelineIndex =
            -1;
    private int lastFiducialCount;
    private double lastStalenessMs;
    private double lastCaptureLatencyMs;
    private double lastTargetingLatencyMs;
    private double lastTotalLatencyMs;
    private String lastPipelineType =
            "unknown";
    private String lastFiducialIds =
            "none";
    private String lastReadError =
            "none";
    private String lastStatusError =
            "none";
    private LimelightTarget lastTarget =
            LimelightTarget.inactive("not sampled");

    public VisionSubsystem(HardwareMap hardwareMap) {

        try {
            limelight =
                    hardwareMap.get(
                            Limelight3A.class,
                            Constants.Hardware.LIMELIGHT_NAME
                    );

            limelight.setPollRateHz(Constants.Vision.POLL_RATE_HZ);
            limelight.pipelineSwitch(Constants.Vision.PIPELINE_INDEX);
            limelight.start();
            available = true;
        } catch (RuntimeException e) {
            available = false;
            limelight = null;
            lastTarget =
                    LimelightTarget.inactive("limelight missing");
        }
    }

    public void update(
            boolean correctionEnabled,
            FieldCoordinates.AllianceGoal selectedGoal
    ) {

        updateStatus();

        if (!correctionEnabled) {
            lastSampleAttempted = false;
            lastResultPresent = false;
            lastResultValid = false;
            lastFiducialCount = 0;
            lastFiducialIds = "none";
            lastReadError = "none";
            lastTarget =
                    LimelightTarget.inactive("correction off");
            return;
        }

        if (!available || limelight == null) {
            connected = false;
            lastSampleAttempted = false;
            lastResultPresent = false;
            lastResultValid = false;
            lastFiducialCount = 0;
            lastFiducialIds = "none";
            lastTarget =
                    LimelightTarget.inactive("limelight unavailable");
            return;
        }

        lastSampleAttempted = true;
        lastReadError = "none";

        LLResult result;

        try {
            result = limelight.getLatestResult();
        } catch (RuntimeException e) {
            lastResultPresent = false;
            lastResultValid = false;
            lastFiducialCount = 0;
            lastFiducialIds = "none";
            lastReadError = e.getClass().getSimpleName();
            lastTarget =
                    LimelightTarget.inactive("read error");
            return;
        }

        lastResultPresent =
                result != null;

        if (result == null || !result.isValid()) {
            lastResultValid = false;
            updateResultDiagnostics(result);
            lastTarget =
                    LimelightTarget.inactive("no valid result");
            return;
        }

        lastResultValid = true;
        updateResultDiagnostics(result);

        if (!limelight.isConnected()) {
            connected = false;
            lastTarget =
                    LimelightTarget.inactive("disconnected");
            return;
        }

        double totalLatencyMs =
                lastTotalLatencyMs;

        if (lastStalenessMs > Constants.Vision.MAX_STALENESS_MS) {
            lastTarget =
                    LimelightTarget.inactive("stale");
            return;
        }

        if (totalLatencyMs > Constants.Vision.MAX_TOTAL_LATENCY_MS) {
            lastTarget =
                    LimelightTarget.inactive("latency");
            return;
        }

        LLResultTypes.FiducialResult fiducial =
                findSelectedTag(
                        result.getFiducialResults(),
                        selectedGoal.aprilTagId
                );

        if (fiducial == null) {
            lastTarget =
                    LimelightTarget.inactive("selected tag not found");
            return;
        }

        double distance =
                estimateDistanceFromTy(
                        fiducial.getTargetYDegrees()
                );

        lastTarget =
                LimelightTarget.valid(
                        selectedGoal.aprilTagId,
                        fiducial.getTargetXDegrees(),
                        fiducial.getTargetYDegrees(),
                        distance,
                        MathUtil.isFinite(distance),
                        totalLatencyMs
                );
    }

    public LimelightTarget getLastTarget() {

        return lastTarget;
    }

    public boolean isReadyToShoot() {

        return lastTarget.valid
                && Math.abs(lastTarget.txDegrees)
                <= Constants.Vision.TX_READY_TOLERANCE_DEGREES;
    }

    public boolean isAvailable() {

        return available;
    }

    public boolean isConnected() {

        return connected;
    }

    public boolean wasLastSampleAttempted() {

        return lastSampleAttempted;
    }

    public boolean wasLastResultPresent() {

        return lastResultPresent;
    }

    public boolean wasLastResultValid() {

        return lastResultValid;
    }

    public int getLastPipelineIndex() {

        return lastPipelineIndex;
    }

    public String getLastPipelineType() {

        return lastPipelineType;
    }

    public int getLastFiducialCount() {

        return lastFiducialCount;
    }

    public String getLastFiducialIds() {

        return lastFiducialIds;
    }

    public double getLastStalenessMs() {

        return lastStalenessMs;
    }

    public double getLastCaptureLatencyMs() {

        return lastCaptureLatencyMs;
    }

    public double getLastTargetingLatencyMs() {

        return lastTargetingLatencyMs;
    }

    public double getLastTotalLatencyMs() {

        return lastTotalLatencyMs;
    }

    public String getLastReadError() {

        return lastReadError;
    }

    public String getLastStatusError() {

        return lastStatusError;
    }

    public void stop() {

        if (limelight != null) {
            try {
                limelight.stop();
            } catch (RuntimeException ignored) {
            }
        }
    }

    private LLResultTypes.FiducialResult findSelectedTag(
            List<LLResultTypes.FiducialResult> fiducials,
            int tagId
    ) {

        if (fiducials == null) {
            return null;
        }

        LLResultTypes.FiducialResult best =
                null;

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() != tagId) {
                continue;
            }

            if (best == null
                    || Math.abs(fiducial.getTargetXDegrees())
                    < Math.abs(best.getTargetXDegrees())) {
                best = fiducial;
            }
        }

        return best;
    }

    private void updateStatus() {

        if (!available || limelight == null) {
            connected = false;
            lastStatusError = "unavailable";
            return;
        }

        try {
            connected =
                    limelight.isConnected();

            LLStatus status =
                    limelight.getStatus();

            if (status != null) {
                lastPipelineIndex =
                        status.getPipelineIndex();
                lastPipelineType =
                        status.getPipelineType();
                lastStatusError = "none";
            } else {
                lastStatusError = "null status";
            }
        } catch (RuntimeException e) {
            connected = false;
            lastStatusError =
                    e.getClass().getSimpleName();
        }
    }

    private void updateResultDiagnostics(LLResult result) {

        if (result == null) {
            lastStalenessMs = 0;
            lastCaptureLatencyMs = 0;
            lastTargetingLatencyMs = 0;
            lastTotalLatencyMs = 0;
            lastFiducialCount = 0;
            lastFiducialIds = "none";
            return;
        }

        lastStalenessMs =
                result.getStaleness();
        lastCaptureLatencyMs =
                result.getCaptureLatency();
        lastTargetingLatencyMs =
                result.getTargetingLatency();
        lastTotalLatencyMs =
                lastStalenessMs
                        + lastCaptureLatencyMs
                        + lastTargetingLatencyMs;

        updateFiducialDiagnostics(
                result.getFiducialResults()
        );
    }

    private void updateFiducialDiagnostics(
            List<LLResultTypes.FiducialResult> fiducials
    ) {

        if (fiducials == null || fiducials.isEmpty()) {
            lastFiducialCount = 0;
            lastFiducialIds = "none";
            return;
        }

        lastFiducialCount =
                fiducials.size();

        StringBuilder ids =
                new StringBuilder();

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (ids.length() > 0) {
                ids.append(",");
            }

            ids.append(fiducial.getFiducialId());
        }

        lastFiducialIds =
                ids.toString();
    }

    private double estimateDistanceFromTy(double targetYDegrees) {

        double totalAngleRadians =
                Math.toRadians(
                        Constants.Vision.LIMELIGHT_MOUNT_PITCH_DEGREES
                                + targetYDegrees
                );

        double tangent =
                Math.tan(totalAngleRadians);

        if (Math.abs(tangent) < 0.001) {
            return Double.NaN;
        }

        double distance =
                (Constants.Vision.GOAL_HEIGHT_INCHES
                        - Constants.Vision.LIMELIGHT_LENS_HEIGHT_INCHES)
                        / tangent;

        if (!MathUtil.isFinite(distance)
                || distance < Constants.Vision.MIN_DISTANCE_INCHES
                || distance > Constants.Vision.MAX_DISTANCE_INCHES) {
            return Double.NaN;
        }

        return distance;
    }
}
