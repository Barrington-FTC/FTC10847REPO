package org.firstinspires.ftc.teamcode.Services;

public class savedPositionService {
    public static double x;//inches
    public static double y;//inches
    public static double heading;//radians
    public static int turretPos;// ticks

    public void SavePosition(double x,double y,double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
    public static void setX(double xval){x = xval;}
    public static void sety(double yval){y = yval;}
    public static void seth(double hval){heading = hval;}
    public static void setTurretPos(int pos){turretPos = pos;}
    public static double getX() {
        return x;
    }
    public static double getY() {
        return y;
    }
    public static double getHeading() {
        return heading;
    }
    public static int getTurretPos(){return turretPos;}
}
