package org.firstinspires.ftc.teamcode.Services;

/**
 * PIDF Controller for moving a motor to a target position.
 *
 * Usage example:
 *   PIDFController pidf = new PIDFController(0.01, 0.0001, 0.0005, 0.1);
 *   pidf.setTargetPosition(500);
 *
 *   // In your loop:
 *   double power = pidf.calculate(motor.getCurrentPosition());
 *   motor.setPower(power);
 */
public class PIDFController {

    // ── Gains ────────────────────────────────────────────────────────────────
    private double kP;
    private double kI;
    private double kD;
    private double kF; // Feedforward (e.g. gravity compensation)

    // ── Targets & State ──────────────────────────────────────────────────────
    private double targetPosition = 0;
    private double lastError      = 0;
    private double integralSum    = 0;
    private long   lastTimeMs     = -1;

    // ── Limits ───────────────────────────────────────────────────────────────
    private double maxOutput        =  1.0;
    private double minOutput        = -1.0;
    private double integralLimit    = Double.MAX_VALUE; // anti-windup clamp
    private double tolerance        = 10.0;  // ticks — "close enough"
    private double derivativeFilter = 0.0;   // 0 = off, 0–1 = low-pass weight

    // ── Internal derivative filtering ────────────────────────────────────────
    private double filteredDerivative = 0;

    // ─────────────────────────────────────────────────────────────────────────
    // Constructors
    // ─────────────────────────────────────────────────────────────────────────

    /** Full constructor. */
    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /** PID only (no feedforward). */
    public PIDFController(double kP, double kI, double kD) {
        this(kP, kI, kD, 0.0);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Core Calculate Method
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Call this every loop iteration with the motor's current encoder position.
     *
     * @param currentPosition  encoder ticks from motor.getCurrentPosition()
     * @return                 power value clamped to [minOutput, maxOutput]
     */
    public double calculate(double currentPosition) {
        long nowMs = System.currentTimeMillis();

        double error = targetPosition - currentPosition;

        // ── Delta time ───────────────────────────────────────────────────────
        double dt = (lastTimeMs < 0) ? 0.02 : Math.max((nowMs - lastTimeMs) / 1000.0, 1e-6);
        lastTimeMs = nowMs;

        // ── Proportional ─────────────────────────────────────────────────────
        double P = kP * error;

        // ── Integral (with anti-windup clamp) ────────────────────────────────
        integralSum += error * dt;
        integralSum  = clamp(integralSum, -integralLimit, integralLimit);
        double I = kI * integralSum;

        // ── Derivative (with optional low-pass filter) ───────────────────────
        double rawDerivative = (dt > 0) ? (error - lastError) / dt : 0;
        if (derivativeFilter > 0 && derivativeFilter < 1) {
            filteredDerivative = derivativeFilter * filteredDerivative
                    + (1 - derivativeFilter) * rawDerivative;
        } else {
            filteredDerivative = rawDerivative;
        }
        double D = kD * filteredDerivative;

        // ── Feedforward ──────────────────────────────────────────────────────
        double F = kF * targetPosition;

        // ── Sum & clamp ──────────────────────────────────────────────────────
        double output = P + I + D + F;
        lastError = error;

        return clamp(output, minOutput, maxOutput);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // State Management
    // ─────────────────────────────────────────────────────────────────────────

    /** Resets integral, derivative memory, and the timer. Call when re-enabling or changing targets. */
    public void reset() {
        integralSum        = 0;
        lastError          = 0;
        filteredDerivative = 0;
        lastTimeMs         = -1;
    }

    /** Returns true if |error| ≤ tolerance. */
    public boolean atTarget() {
        return Math.abs(targetPosition - lastError - targetPosition + lastError) <= tolerance
                || Math.abs(lastError) <= tolerance;
    }

    /** Convenience: resets and sets a new target in one call. */
    public void setTargetAndReset(double targetPosition) {
        reset();
        this.targetPosition = targetPosition;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Getters & Setters — Gains
    // ─────────────────────────────────────────────────────────────────────────

    public double getKP() { return kP; }
    public void   setKP(double kP) { this.kP = kP; }

    public double getKI() { return kI; }
    public void   setKI(double kI) { this.kI = kI; }

    public double getKD() { return kD; }
    public void   setKD(double kD) { this.kD = kD; }

    public double getKF() { return kF; }
    public void   setKF(double kF) { this.kF = kF; }

    /** Set all four gains at once. */
    public void setGains(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Getters & Setters — Target
    // ─────────────────────────────────────────────────────────────────────────

    public double getTargetPosition() { return targetPosition; }
    public void   setTargetPosition(double targetPosition) { this.targetPosition = targetPosition; }

    // ─────────────────────────────────────────────────────────────────────────
    // Getters & Setters — Output limits
    // ─────────────────────────────────────────────────────────────────────────

    public double getMaxOutput() { return maxOutput; }
    public void   setMaxOutput(double maxOutput) { this.maxOutput = maxOutput; }

    public double getMinOutput() { return minOutput; }
    public void   setMinOutput(double minOutput) { this.minOutput = minOutput; }

    /** Symmetric output clamp: sets max to +limit and min to -limit. */
    public void setOutputRange(double limit) {
        this.maxOutput =  Math.abs(limit);
        this.minOutput = -Math.abs(limit);
    }

    /** Asymmetric output clamp. */
    public void setOutputRange(double min, double max) {
        this.minOutput = min;
        this.maxOutput = max;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Getters & Setters — Integral limit (anti-windup)
    // ─────────────────────────────────────────────────────────────────────────

    public double getIntegralLimit() { return integralLimit; }
    public void   setIntegralLimit(double integralLimit) { this.integralLimit = Math.abs(integralLimit); }

    // ─────────────────────────────────────────────────────────────────────────
    // Getters & Setters — Tolerance
    // ─────────────────────────────────────────────────────────────────────────

    public double getTolerance() { return tolerance; }
    public void   setTolerance(double tolerance) { this.tolerance = Math.abs(tolerance); }

    // ─────────────────────────────────────────────────────────────────────────
    // Getters & Setters — Derivative filter
    // ─────────────────────────────────────────────────────────────────────────

    /** Weight for low-pass derivative filter. 0 = disabled, closer to 1 = more smoothing. */
    public double getDerivativeFilter() { return derivativeFilter; }
    public void   setDerivativeFilter(double alpha) {
        this.derivativeFilter = clamp(alpha, 0.0, 0.99);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Read-only state accessors
    // ─────────────────────────────────────────────────────────────────────────

    /** Current error: target − last measured position. */
    public double getLastError()          { return lastError; }

    /** Accumulated integral sum (before kI scaling). */
    public double getIntegralSum()        { return integralSum; }

    /** Filtered derivative value (before kD scaling). */
    public double getFilteredDerivative() { return filteredDerivative; }

    // ─────────────────────────────────────────────────────────────────────────
    // Helpers
    // ─────────────────────────────────────────────────────────────────────────

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public String toString() {
        return String.format(
                "PIDFController{kP=%.4f, kI=%.4f, kD=%.4f, kF=%.4f, target=%.1f, lastError=%.1f}",
                kP, kI, kD, kF, targetPosition, lastError
        );
    }
}