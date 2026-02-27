package org.firstinspires.ftc.teamcode.Services;
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

    public static class turretAimingService {
        private double TargetX = 0;
        private double TargetY = 0;
        private double limeLightAjustment = 0;

        private static final double TURRET_MOTOR_TICKS_PER_REVOLUTION = 145.1;
        // This is the gear ratio between the motor and the turret.
        private static final double TURRET_GEAR_RATIO = 5.0; // Change this to match your gear ratio
        private static final double TURRET_TICKS_PER_RADIAN = (TURRET_MOTOR_TICKS_PER_REVOLUTION * TURRET_GEAR_RATIO) / (2 * Math.PI);
        private double relTargetangle = 0;// angle from from of robot to target calculate later relative to the bot cordnate system
        private double targetangle = 0; // angle from from of robot to target calculate later relative to the field cordnate system
        private int turretTargetPosition;
        private int turretmaxr = 0;
        private int turretmaxl = 370;

        public void initTurretAiming(double x, double y){
            TargetX = x;
            TargetY = y;
        }
        public int aimTurret(double currentX,double currentY,double currentH){
            targetangle = Math.atan2(TargetY - currentY, TargetX - currentX);
            relTargetangle = targetangle - currentH;//angle that we want turret to aim if robot was facing forward - angle robot is currently facing plus angle limelight tells us we are off by.
            relTargetangle = Math.atan2(Math.sin(relTargetangle), Math.cos(relTargetangle));
    // Shift so forward = pi/2
            double turretAngle = relTargetangle + (Math.PI / 2.0);

    // Clamp to turret range
            turretAngle = Math.max(0.0, Math.min(Math.PI, turretAngle));

    // Convert to ticks
            turretTargetPosition = (int)(turretAngle * TURRET_TICKS_PER_RADIAN);
            // Clamp the target position to within the physical limits of the turret
            turretTargetPosition = Math.max(turretmaxr,
                    Math.min(turretmaxl, turretTargetPosition));
            return turretTargetPosition;
        }
    }
}
