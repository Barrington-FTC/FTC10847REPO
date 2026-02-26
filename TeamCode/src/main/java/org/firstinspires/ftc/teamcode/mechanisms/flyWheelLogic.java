package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PIDService;

public class flyWheelLogic {
    private DcMotorEx flyWheelR = null;
    private DcMotorEx flyWheelL = null;

    private DcMotorEx intake = null;
    private Servo lAngle = null;
    private Servo blocker = null;

    private ElapsedTime stateTimer = new ElapsedTime();

    private PIDService pidService = new PIDService();

    private IntakeLogic intaker = new IntakeLogic();
    public enum FlywheelState {
        IDLE,
        SPIN_UP,
        Shoot
    }

    private FlywheelState flywheelState;
    // constants
    private double GATE_CLOSE_ANGLE = 0.9;
    private double GATE_OPEN_ANGLE = 1;

    private double GATE_OPEN_TIME = .5; // seconds

    // flywheel constants
    private int shotsRemaining = 0;

    private double TARGET_FLYWHEEL_VELOCITY = 0;//norammly 1700

    public void init(HardwareMap hwMap, Servo Blocker,DcMotorEx Intake,int targetVelocity) {
        flyWheelR = hwMap.get(DcMotorEx.class, "flyWheelR");
        flyWheelL = hwMap.get(DcMotorEx.class, "flyWheelL");
        lAngle = hwMap.get(Servo.class, "lAngle");
        blocker = Blocker;
        intake = Intake;
        TARGET_FLYWHEEL_VELOCITY = targetVelocity;
        flyWheelR.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheelL.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheelL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients coefficients = new PIDFCoefficients(pidService.getFinalKP(),.001, pidService.getFlywheeKD(), pidService.getFinalKF());
        flyWheelR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        flyWheelL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);     // turret setup
        lAngle.setPosition(0);
        flywheelState = FlywheelState.IDLE;
    }

    public void update() {
        flyWheelL.setVelocity(TARGET_FLYWHEEL_VELOCITY);
        flyWheelR.setVelocity(TARGET_FLYWHEEL_VELOCITY);
        switch (flywheelState) {
            case IDLE:
                if(shotsRemaining>0){
                    stateTimer.reset();
                    blocker.setPosition(.9);
                    flywheelState = FlywheelState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                intake.setPower(0);
                if(flyWheelL.getVelocity()<-(TARGET_FLYWHEEL_VELOCITY-50)){
                    blocker.setPosition(1);
                    shotsRemaining--;
                    stateTimer.reset();
                    flywheelState = FlywheelState.Shoot;

                }
                break;
            case Shoot:
                if(stateTimer.seconds()>.25 && blocker.getPosition()==GATE_OPEN_ANGLE){
                    intake.setPower(1);
                    if(stateTimer.seconds()>.75){
                        if(shotsRemaining <= 0){
                            intake.setPower(0);
                            blocker.setPosition(.9);
                            stateTimer.reset();
                            flywheelState = FlywheelState.IDLE;
                        }
                    else{
                        stateTimer.reset();
                        flywheelState = FlywheelState.SPIN_UP;
                    }
                    }
                }
                else{
                    intake.setPower(0);
                }
                break;
        }

    }

    public void fireShots(int numShots) {
        if (flywheelState == FlywheelState.IDLE) {
            shotsRemaining = numShots;
        }
    }

    public boolean IDLE() {
        return flywheelState == FlywheelState.IDLE;
    }

    public int getShotsRemaining() {
        return shotsRemaining;
    }

    public FlywheelState getFlywheelState() {
        return flywheelState;
    }
    public void forceCloseGate(){
        blocker.setPosition(GATE_CLOSE_ANGLE);
    }
}
