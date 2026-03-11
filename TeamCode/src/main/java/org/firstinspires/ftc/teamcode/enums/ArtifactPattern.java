package org.firstinspires.ftc.teamcode.enums;

public enum ArtifactPattern {
    PGP(1, 3), GPP(2), PPG(4);

    public final int[] spikes;

    ArtifactPattern(int... spikes) {
        this.spikes = spikes;
    }

    public static ArtifactPattern fromSpike(int spike) {
        for (ArtifactPattern p : values())
            for (int s : p.spikes) if (s == spike) return p;
        throw new IllegalArgumentException("No pattern for spike " + spike);
    }

    /** How many sorts needed to become the target pattern */
    public int sortsTo(ArtifactPattern target) {
        ArtifactPattern[] cycle = { PPG, GPP, PGP };
        int from = indexOf(cycle, this);
        int to = indexOf(cycle, target);
        return (to - from + cycle.length) % cycle.length;
    }

    private static int indexOf(ArtifactPattern[] arr, ArtifactPattern p) {
        for (int i = 0; i < arr.length; i++) if (arr[i] == p) return i;
        return -1;
    }
}
