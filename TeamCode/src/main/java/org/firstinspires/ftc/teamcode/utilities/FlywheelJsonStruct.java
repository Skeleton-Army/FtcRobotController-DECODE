package org.firstinspires.ftc.teamcode.utilities;

public class FlywheelJsonStruct {
    private double tps;
    private double rpm;
    private double error;
    private double recovery;
    private double amps;
    private double voltage;
    private double runtime;

    public FlywheelJsonStruct(double tps, double rpm, double error, double recovery, double amps, double voltage, double runtime) {
        this.tps = tps;
        this.rpm = rpm;
        this.error = error;
        this.recovery = recovery;
        this.amps = amps;
        this.voltage = voltage;
        this.runtime = runtime;
    }

    public double getTps() {
        return tps;
    }

    public void setTps(double tps) {
        this.tps = tps;
    }

    public double getRpm() {
        return rpm;
    }

    public void setRpm(double rpm) {
        this.rpm = rpm;
    }

    public double getError() {
        return error;
    }

    public void setError(double error) {
        this.error = error;
    }

    public double getRecovery() {
        return recovery;
    }

    public void setRecovery(double recovery) {
        this.recovery = recovery;
    }

    public double getAmps() {
        return amps;
    }

    public void setAmps(double amps) {
        this.amps = amps;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    public double getVoltage() {
        return voltage;
    }

    public double getRuntime() {
        return runtime;
    }

    public void setRuntime(double runtime) {
        this.runtime = runtime;
    }
}
