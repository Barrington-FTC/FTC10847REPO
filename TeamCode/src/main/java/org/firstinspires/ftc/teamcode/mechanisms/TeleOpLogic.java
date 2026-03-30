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
        flyWheelR.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheelL.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheelL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        Blocker = hwMap.get(Servo.class,"blocker");
        Blocker.setDirection(Servo.Direction.FORWARD);

        flyWheelR.setVelocityPIDFCoefficients(PIDservice.getFinalKP(),0,0,PIDservice.getFinalKP());
        flyWheelL.setVelocityPIDFCoefficients(PIDservice.getFinalKP(),0,0,PIDservice.getFinalKP());
        Blocker.setPosition(.9);

        OpState = TeleOpLogic.state.IDLE;
    }

    public void update(double TARGET_VELOCITY) {
        switch (OpState) {
            case IDLE:
                flyWheelL.setVelocity(TARGET_VELOCITY);
                flyWheelR.setVelocity(TARGET_VELOCITY);
                Blocker.setPosition(GATE_CLOSE_ANGLE);
                if(shotsRemaining>0 && (Math.abs(flyWheelL.getVelocity())>=(TARGET_VELOCITY-20) && Math.abs(flyWheelL.getVelocity())<=(TARGET_VELOCITY+20))){
                    stateTimer.reset();
                    OpState = TeleOpLogic.state.Shooting;
                }
                break;
            case Shooting:
                Blocker.setPosition(GATE_OPEN_ANGLE);
                if(stateTimer.seconds()>=Gate_Toggle_Time){
                    stateTimer.reset();
                    OpState = TeleOpLogic.state.Shooting;
                }
                break;

            case RECOVERY:
                if(stateTimer.seconds()<CylceTime){
                    if(Math.abs(flyWheelL.getVelocity())<(TARGET_VELOCITY-40)){
                        flyWheelL.setPower(1);
                        flyWheelR.setPower(1);
                    }
                    else{
                        flyWheelL.setVelocity(TARGET_VELOCITY);
                        flyWheelR.setVelocity(TARGET_VELOCITY);
                    }
                }
                else{
                    shotsRemaining = 0;
                    stateTimer.reset();
                    OpState = TeleOpLogic.state.IDLE;
                }
                break;
        }

    }
    public void setShotsRemaining(int shotsRemaining){
        this.shotsRemaining = shotsRemaining;
    }
}
