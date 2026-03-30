package org.firstinspires.ftc.teamcode.Services;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class DualPIDF {
    // Coefficients
    private double pKp, pKi, pKd, pKf;
    private double vKp, vKi, vKd, vKf;

    private double posErrorSum, velErrorSum;
    private double lastActualPos, lastActualVel;
    private double iSumMax = 0.20; // Cap "I" contribution to 25% motor power

    private ElapsedTime timer = new ElapsedTime();
    private double maxVelocity;
    private boolean firstRun = true;

    public DualPIDF(double maxVel) {
        this.maxVelocity = maxVel;
        timer.reset();
    }

    public void setPositionCoefficients(double p, double i, double d, double f) {
        this.pKp = p; this.pKi = i; this.pKd = d; this.pKf = f;
    }

    public void setVelocityCoefficients(double p, double i, double d, double f) {
        this.vKp = p; this.vKi = i; this.vKd = d; this.vKf = f;
    }

    /**
     * Set the maximum power the Integral term can contribute (0.0 to 1.0)
     */
    public void setIntegralCap(double cap) {
        this.iSumMax = cap;
    }

    public double updateVelocity(double targetVel, double currentVel) {
        double dt = timer.seconds();
        if (dt == 0) return 0;
        timer.reset();

        if (firstRun) {
            lastActualVel = currentVel;
            firstRun = false;
        }

        double velError = targetVel - currentVel;

        // 1. Integral Anti-Windup (Cap the sum)
        velErrorSum = velError * dt;
        double iTerm = velErrorSum * vKi;
        iTerm = Range.clip(iTerm, -iSumMax, iSumMax);

        // 2. Derivative on Measurement (Change in actual velocity)
        // Note: We use negative here because if actual velocity increases,
        // it should resist the motor power (acting as a damper).
        double velDeriv = (lastActualVel - currentVel) / dt;

        double motorPower = (velError * vKp) + iTerm + (velDeriv * vKd) + (targetVel * vKf);

        lastActualVel = currentVel;
        return Range.clip(motorPower, -1.0, 1.0);
    }

    public double updatePosition(double targetPos, double currentPos, double currentVel) {
        double dt = timer.seconds();
        if (dt == 0) return 0;

        if (firstRun) {
            lastActualPos = currentPos;
            // Note: firstRun will be set to false inside updateVelocity()
        }

        double posError = targetPos - currentPos;

        // 1. Position Integral Cap
        posErrorSum = posError * dt;
        double iTerm = posErrorSum * pKi;
        iTerm = Range.clip(iTerm, -iSumMax, iSumMax);

        // 2. Position Derivative on Measurement
        double posDeriv = (lastActualPos - currentPos) / dt;

        double commandedVelocity = (posError * pKp) + iTerm + (posDeriv * pKd) + (targetPos * pKf);
        commandedVelocity = Range.clip(commandedVelocity, -maxVelocity, maxVelocity);

        lastActualPos = currentPos;

        // Pass calculated velocity target to the velocity PIDF
        return updateVelocity(commandedVelocity, currentVel);
    }

    public void reset() {
        posErrorSum = 0;
        velErrorSum = 0;
        lastActualPos = 0;
        lastActualVel = 0;
        firstRun = true;
        timer.reset();
    }
}