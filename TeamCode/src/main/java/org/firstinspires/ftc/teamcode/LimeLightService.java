package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimeLightService {
    private Limelight3A limelight= null;


    private double tx;// How far left or right the target is (degrees)

    private double ty;// How far up or down the target is (degrees)

    private double ta; // How big the target looks (0%-100% of the image)


    private double x;


    private double y;


    private double heading;

    public void initLimeLight(HardwareMap hw){
        limelight = hw.get(Limelight3A.class,"limeLight");
        limelight.setPollRateHz(90);
        limelight.start();
    }
    public void UpdateLimeLightData(){
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                x = botpose.getPosition().x;
                y = botpose.getPosition().y;
                heading = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
            }
            tx = result.getTx();
            ty = result.getTy();
            ta = result.getTa();
        }
    }
    public double getTx() {
        return tx;
    }
    public double getTy() {
        return ty;
    }
    public double getTa() {
        return ta;
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public void setLimeLightFilter(int filter){
        limelight.pipelineSwitch(filter);
    }
    public double getHeading() {
        return heading;
    }

}
