package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Services.PIDService;

public class TeleOpLogic {
    private DcMotorEx flyWheelR = null;
    private DcMotorEx flyWheelL = null;
    public Servo Blocker;


    private ElapsedTime stateTimer = new ElapsedTime();

    private PIDService PIDservice = new PIDService();
    private double CylceTime = 1.5;//in seconds

    public enum state {
        IDLE,
        Shooting,
        RECOVERY

    }

    private state OpState;
    // constants
    private double GATE_CLOSE_ANGLE = 0.9;
    private double GATE_OPEN_ANGLE = 1;
    private double Gate_Toggle_Time = .5; // seconds

    // flywheel constants
    private int shotsRemaining = 0;

    public void init(HardwareMap hwMap) {

        flyWheelR = hwMap.get(DcMotorEx.class, "flyWheelR");
        flyWheelL = hwMap.get(DcMotorEx.class, "flyWheelL");
        flyWheelR.setDirection(DcMotorEx.Direction.FORWARD);
        flyWheelR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyWheelL.setDirection(DcMotorEx.Direction.REVERSE);
        flyWheelL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        Blocker = hwMap.get(Servo.class,"blocker");
        Blocker.setDirection(Servo.Direction.FORWARD);

        flyWheelR.setVelocityPIDFCoefficients(PIDservice.getFinalKP(),0,0,PIDservice.getFinalKP());
        flyWheelL.setVelocityPIDFCoefficients(PIDservice.getFinalKP(),0,0,PIDservice.getFinalKP());
        Blocker.setPosition(.9);

        OpState = TeleOpLogic.state.IDLE;
    }

    public void update(double TARGET_VELOCITY) {
        if(Math.abs(flyWheelL.getVelocity())<TARGET_VELOCITY-20){
            flyWheelL.setPower(1);
            flyWheelR.setPower(1);
        }
        else{
            flyWheelL.setVelocity(TARGET_VELOCITY);
            flyWheelR.setVelocity(TARGET_VELOCITY);
        }
        switch (OpState) {
            case IDLE:
                Blocker.setPosition(GATE_CLOSE_ANGLE);
                if(shotsRemaining>0){
                    stateTimer.reset();
                    OpState = state.Shooting;
                }
                break;
            case Shooting:
                Blocker.setPosition(GATE_OPEN_ANGLE);
                if(stateTimer.seconds()>=1.5){
                    shotsRemaining = 0;
                    stateTimer.reset();
                    OpState = state.IDLE;
                }
                break;
        }

    }
    public void setShotsRemaining(int shotsRemaining){
        this.shotsRemaining = shotsRemaining;
    }
    public double getFlPow(){
        return flyWheelL.getPower();
    }
    public double getFRPow(){
        return flyWheelR.getPower();
    }
}
