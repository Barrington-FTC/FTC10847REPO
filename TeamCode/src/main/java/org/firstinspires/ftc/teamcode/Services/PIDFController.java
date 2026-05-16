package org.firstinspires.ftc.teamcode.Services;


//used for velocity control
public class PIDFController {

    private double kp;
    private double ki;
    private double kd;
    private double kf;

    private double targetVelocity;
    private double integralSum;
    private double lastError;
    private long lastTime;

    private double Velocity;
    private int Last_Position = 0;
    private int Current_Position = 0;
    private double Current_Time;
    private double Previous_Time;
    public void calculate_velocity(int pos, double time){
        Last_Position = Current_Position;
        Current_Position = pos;
        Previous_Time = Current_Time;
        Current_Time = time;
        Velocity = (double) (Current_Position - Last_Position) / (Current_Time - Previous_Time);
    }

    /**
     * Constructor for the PIDF Controller.
     */
    public PIDFController(double kp, double ki, double kd, double kf) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;

        this.targetVelocity = 0;
        this.integralSum = 0;
        this.lastError = 0;
        this.lastTime = System.nanoTime();
    }

    /**
     * Calculates the motor power output based on the current velocity.
     * Run this continuously in your main loop.
     * * @param currentVelocity The current velocity reading from the motor encoder.
     * @return The calculated power output to send to the motor.
     */
    public double calculate(double currentVelocity) {
        long currentTime = System.nanoTime();
        // Convert nanoseconds to seconds for precise time delta
        double dt = (currentTime - lastTime) / 1.0E9;

        // Prevent division by zero on the first loop
        if (dt == 0) {
            dt = 0.001;
        }

        double error = targetVelocity - currentVelocity;

        if(error>100){
            return 1;
        }

        // Proportional
        double p = (kp/100) * error;

        // Integral (with basic windup prevention)
        integralSum += (error * dt);
        double i = (ki/1000) * integralSum;

        // Derivative
        double derivative = (error - lastError) / dt;
        double d = (kd/100) * derivative;

        // Feedforward (Based on the target velocity, not the error)
        double f = (kf/1000) * targetVelocity;

        // Save state for the next loop
        lastError = error;
        lastTime = currentTime;

        double output = p + i + d + f;

        return Math.max(0, Math.min(1.0, output));
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
        this.kf = kf;
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public void setKp(double kp) { this.kp = kp; }
    public void setKi(double ki) { this.ki = ki; }
    public void setKd(double kd) { this.kd = kd; }
    public void setKf(double kf) { this.kf = kf; }

    // --- Getters ---

    public double getTargetVelocity() { return targetVelocity; }
    public double getKp() { return kp; }
    public double getKi() { return ki; }
    public double getKd() { return kd; }
    public double getKf() { return kf; }
    public double getVelocity(){return Velocity;}
}