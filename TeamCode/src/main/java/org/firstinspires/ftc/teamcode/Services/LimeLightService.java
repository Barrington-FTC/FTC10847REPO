package org.firstinspires.ftc.teamcode.Services;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.apache.commons.math3.stat.regression.SimpleRegression;

import java.util.ArrayList;
import java.util.List;

public class LimeLightService {
    private Limelight3A limelight= null;

    private double limlightHight = 24;//inches

    private double targetHight = 1.45;//inches

    private double cameraAngle = -0.1; // radians


    private double tx;// How far left or right the target is (degrees)

    private double ty;// How far up or down the target is (degrees)

    private double ta; // How big the target looks (0%-100% of the image)


    private double x;


    private double y;


    private double heading;

    private List<Integer> Targets = new ArrayList<>();
    private List<Double> TargetsX = new ArrayList<>();
    private List<Double> TargetsY = new ArrayList<>();

    public void initLimeLight(HardwareMap hw){
        limelight = hw.get(Limelight3A.class,"limeLight");
        limelight.setPollRateHz(90);
        limelight.start();
    }
    public void updatePosData(){
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
    public void SearchForElements(double xPos,double yPos, double heading){
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        Targets.clear();
        TargetsX.clear();
        TargetsY.clear();
        for (LLResultTypes.DetectorResult detection : detections) {
            double x = detection.getTargetXDegrees(); // Where it is (left-right)
            double y = detection.getTargetYDegrees(); // Where it is (up-down)
            double forwardDistance = (limlightHight-targetHight)/Math.tan((Math.toRadians(y)+cameraAngle)*(Math.PI/180));
            double horizontalDistance = forwardDistance*Math.tan(Math.toRadians(x)*(Math.PI/180));
            double radius = Math.sqrt(Math.pow(horizontalDistance,2)+Math.pow(forwardDistance,2));
            TargetsX.add(xPos + radius*Math.cos(Math.toRadians(heading)));
            TargetsY.add(yPos + radius*Math.sin(Math.toRadians(heading)));
            Targets.add(detection.getClassId());
        }
    }
    /*
    public double regression(){
        SimpleRegression regression = new SimpleRegression();
        regression.addData(1, 2);

        regression.addData(1, 2);
        regression.addData(2, 3);
        regression.addData(3, 5);
        regression.addData(4, 4);
        regression.addData(5, 6);
        regression.
    }

     */
    public double calculateAverageX(){
        double avg = 0;
        for(int i = 0; i<TargetsX.size();i++){
            avg += TargetsX.get(i);

        }
        avg/=TargetsX.size();
        return avg;
    }
    public double calculateAverageY(){
        double avg = 0;
        for(int i = 0; i<TargetsY.size();i++){
            avg += TargetsY.get(i);

        }
        avg/=TargetsY.size();
        return avg;
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
