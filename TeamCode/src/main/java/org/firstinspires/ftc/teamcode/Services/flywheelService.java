package org.firstinspires.ftc.teamcode.Services;
import org.firstinspires.ftc.teamcode.Services.PIDController;
import org.firstinspires.ftc.teamcode.Services.PIDFController;

public class flywheelService {
    public double distanceCoefficent = 4.35;
    public double distanceConstant = 1270.24468;

    public double TargetVelocity;
    private PIDService PIDservice = new PIDService();
    private PIDFController pidfController = new PIDFController(PIDservice.getFinalKP(),0,0,PIDservice.getFinalKF());

    public void calculateFlywheelTargetVelocity(double Distance){
        TargetVelocity = distanceCoefficent * Distance + distanceConstant;
    }

    public double getDistanceCoefficent() {
        return distanceCoefficent;
    }

    public void setDistanceCoefficent(double distanceCoefficent) {
        this.distanceCoefficent = distanceCoefficent;
    }


    public double getDistanceConstant() {
        return distanceConstant;
    }

    public void setDistanceConstant(double distanceConstant) {
        this.distanceConstant = distanceConstant;
    }
    public double RunFlywheel(){
        double flyhweelVelocity = pidfController.getVelocity();
        double targetVelocity = TargetVelocity;
        pidfController.setTargetVelocity(targetVelocity);
        return pidfController.calculate(flyhweelVelocity);

    }




}
