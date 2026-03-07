package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Services.PIDService;

public class AutoLogic {
    private DcMotorEx flyWheelR = null;
    private DcMotorEx flyWheelL = null;
    private Servo lAngle = null;
    public Servo Blocker;
    public DcMotorEx Intake;


    private ElapsedTime stateTimer = new ElapsedTime();

    private PIDService pidService = new PIDService();
    public enum state {
        IDLE,
        SPIN_UP,
        SHOOT,
        INTAKE

    }

    private state AutoState;
    // constants
    private double GATE_CLOSE_ANGLE = 0.9;
    private double GATE_OPEN_ANGLE = 1;

    private double GATE_OPEN_TIME = .5; // seconds

    // flywheel constants
    private int shotsRemaining = 0;

    private int ballsRemainig = 0;

    private double TARGET_FLYWHEEL_VELOCITY = 0;

    public void init(HardwareMap hwMap,int targetVelocity) {

        flyWheelR = hwMap.get(DcMotorEx.class, "flyWheelR");
        flyWheelL = hwMap.get(DcMotorEx.class, "flyWheelL");
        flyWheelR.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheelL.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheelL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        lAngle = hwMap.get(Servo.class, "lAngle");
        lAngle.setDirection(Servo.Direction.FORWARD);

        Blocker = hwMap.get(Servo.class,"blocker");
        Blocker.setDirection(Servo.Direction.FORWARD);

        Intake = hwMap.get(DcMotorEx.class,"intake");
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);

        TARGET_FLYWHEEL_VELOCITY = targetVelocity;


        PIDFCoefficients coefficients = new PIDFCoefficients(0.5,0, 0.02, 13);
        flyWheelR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        flyWheelL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);

        lAngle.setPosition(0);
        Blocker.setPosition(.9);

        AutoState = state.IDLE;
    }

    public void update() {
        flyWheelL.setVelocity(TARGET_FLYWHEEL_VELOCITY);
        flyWheelR.setVelocity(TARGET_FLYWHEEL_VELOCITY);
        switch (AutoState) {
            case IDLE:
                Blocker.setPosition(GATE_CLOSE_ANGLE );
                Intake.setPower(0);
                if(ballsRemainig>0){
                    Intake.setPower(1);
                    stateTimer.reset();
                    AutoState = state.INTAKE;
                }
                if(shotsRemaining>0){
                    Intake.setPower(0);
                    stateTimer.reset();
                    AutoState = state.SPIN_UP;
                }
                break;
            case SPIN_UP:
                Intake.setPower(0);
                if(flyWheelL.getVelocity()<-(TARGET_FLYWHEEL_VELOCITY+50)){
                    Blocker.setPosition(GATE_OPEN_ANGLE);
                    stateTimer.reset();
                    AutoState = state.SHOOT;

                }
                break;
            case SHOOT:
                Intake.setPower(1);
                if(stateTimer.seconds()>.45 && Blocker.getPosition()==GATE_OPEN_ANGLE){
                    Intake.setPower(0);
                    if(stateTimer.seconds()>1){
                        shotsRemaining--;
                    if(shotsRemaining == 0){
                        Intake.setPower(0);
                        Blocker.setPosition(GATE_CLOSE_ANGLE);
                        stateTimer.reset();
                        AutoState = state.IDLE;
                    }
                    else{
                        stateTimer.reset();
                        AutoState = state.SPIN_UP;
                        }
                    }
                }
                break;
            case INTAKE:
                if (stateTimer.seconds() > 5) { // Runs intake for 3 seconds
                    ballsRemainig = 0;
                    Intake.setPower(1);
                    AutoState = state.IDLE;
                }
                break;
        }

    }

    public void fireShots(int numShots) {
        shotsRemaining = numShots;
    }
    public int getShotsremaining(){
        return shotsRemaining;
    }
    public int getintakeremaining(){
        return ballsRemainig;
    }
    public void intakeBalls(){
        ballsRemainig = 1;
    }
    public void setTARGET_FLYWHEEL_VELOCITY(int v){
        TARGET_FLYWHEEL_VELOCITY = v;
    }

    public boolean IDLE() {
        return AutoState == state.IDLE;
    }
}
