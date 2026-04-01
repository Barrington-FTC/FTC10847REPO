package org.firstinspires.ftc.teamcode.Services;
//for Position
public class PIDController {
    private double kp;
    private double ki;
    private double kd;

    private double targetVelocity;
    private double integralSum;
    private double lastError;
    private long lastTime;

    /**
     * Constructor for the PIDF Controller.
     */
    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;

        this.targetVelocity = 0;
        this.integralSum = 0;
        this.lastError = 0;
        this.lastTime = System.nanoTime();
    }

    /**
     * Calculates the motor power output based on the current velocity.
     * Run this continuously in your main loop.
     * * @param currentPosition The current velocity reading from the motor encoder.
     * @return The calculated power output to send to the motor.
     */
    public double calculate(double currentPosition) {
        long currentTime = System.nanoTime();
        // Convert nanoseconds to seconds for precise time delta
        double dt = (currentTime - lastTime) / 1.0E9;

        // Prevent division by zero on the first loop
        if (dt == 0) {
            dt = 0.001;
        }

        double error = targetVelocity - currentPosition;

        // Proportional
        double p = kp/100 * error;

        // Integral (with basic windup prevention)
        integralSum += (error * dt);
        double i = ki/100 * integralSum;

        // Derivative
        double derivative = (error - lastError) / dt;
        double d = kd/100 * derivative;


        // Save state for the next loop
        lastError = error;
        lastTime = currentTime;

        return p + i + d;
    }

    /**
     * Resets the internal state. Call this when starting or restarting the flywheel.
     */
    public void reset() {
        integralSum = 0;
        lastError = 0;
        lastTime = System.nanoTime();
    }

    // --- Setters ---

    public void setPIDF(double kp, double ki, double kd, double kf) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public void setTargetPosition(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public void setKp(double kp) { this.kp = kp; }
    public void setKi(double ki) { this.ki = ki; }
    public void setKd(double kd) { this.kd = kd; }

    // --- Getters ---

    public double getTargetVelocity() { return targetVelocity; }
    public double getKp() { return kp; }
    public double getKi() { return ki; }
    public double getKd() { return kd; }
}


