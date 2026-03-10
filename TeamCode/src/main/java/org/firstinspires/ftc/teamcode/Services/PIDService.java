package org.firstinspires.ftc.teamcode.Services;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class PIDService {
    private double FlyhweelKP = 5;

    private final double FinalKP = 5;
    private final double FlywheeKI = 0;
    private double FlywheeKD = 0;
    private double FlywheeKF = 7;
    private final double FinalKF = 14;


    public PIDFCoefficients getFlywheelCoefficents(){
        PIDFCoefficients flyhwheelconts = new PIDFCoefficients(FinalKP,FlywheeKI,FlywheeKD,FinalKF);
        return flyhwheelconts;
    }
    public void setFlyhweelKP(double kp){
        FlyhweelKP = kp;

    }
    public void setFlyhweelKI(double ki){
        FlyhweelKP = ki;

    }
    public void setFlyhweelKD(double kd){
        FlyhweelKP = kd;

    }
    public void setFlyhweelKF(double kf){
        FlywheeKF = kf;

    }
    public double getFlyhweelKP() {
        return FlyhweelKP;
    }

    public double getFlywheeKI() {
        return FlywheeKI;
    }

    public double getFlywheeKF() {
        return FlywheeKF;
    }

    public double getFlywheeKD() {
        return FlywheeKD;
    }
    public double getFinalKF() {
        return FinalKF;
    }
    public double getFinalKP() {
        return FinalKP;
    }
}
