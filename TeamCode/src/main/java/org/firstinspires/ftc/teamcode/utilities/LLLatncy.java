package org.firstinspires.ftc.teamcode.utilities;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;

public class LLLatncy {
    private final double timestamp;
    private final double timeSinceLastUpdate;
    private final double jsonPrasing;
    private final double captureLatency;
    private final double targetingLatency;

    public LLLatncy(double timestamp, double timeSinceLastUpdate, double jsonLatency, double captureLatency, double targetingLatency) {
        this.timestamp = timestamp;
        this.timeSinceLastUpdate = timeSinceLastUpdate;
        this.jsonPrasing = jsonLatency;
        this.captureLatency = captureLatency;
        this.targetingLatency = targetingLatency;
    }

    public LLLatncy(LLResult llResult, double timeSinceLastUpdate) {
        this.timestamp = llResult.getTimestamp();
        this.timeSinceLastUpdate = timeSinceLastUpdate;
        this.jsonPrasing = llResult.getParseLatency();
        this.captureLatency = llResult.getCaptureLatency();
        this.targetingLatency = llResult.getTargetingLatency();
    }


    public double getTargetingLatency() {
        return targetingLatency;
    }

    public double getCaptureLatency() {
        return captureLatency;
    }

    public double getJsonPrasing() {
        return jsonPrasing;
    }

    public double getTimeSinceLastUpdate() {
        return timeSinceLastUpdate;
    }

    public double getTimestamp() {
        return timestamp;
    }

    public double getTotalLatency() {
        return targetingLatency + captureLatency + jsonPrasing;
    }

    @NonNull
    @SuppressLint("DefaultLocale")
    @Override
    public String toString() {
        return String.format("{timestamp: %f, tlu: %f, parsing: %f, targetings: %f, capture: %f, total: %f",
                timestamp, timeSinceLastUpdate, jsonPrasing, targetingLatency, captureLatency, getTotalLatency());
    }
}
