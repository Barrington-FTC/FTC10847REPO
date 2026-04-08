package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Services.PIDFController;
import org.firstinspires.ftc.teamcode.Services.PIDService;

public class AutoLogic {
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
    private double GATE_CLOSE_ANGLE = 0.90;
    private double GATE_OPEN_ANGLE = 1;

    private double GATE_OPEN_TIME = .5; // seconds

    // flywheel constants
    private int shotsRemaining = 0;

    private int ballsRemainig = 0;

    private double TARGET_FLYWHEEL_VELOCITY = 0;

    private double intakeDuration;
    private PIDService PIDservice = new PIDService();
    private PIDFController pidfController = new PIDFController(PIDservice.getFinalKP(),0,0,PIDservice.getFinalKF());

    public void init(HardwareMap hwMap) {




        lAngle = hwMap.get(Servo.class, "lAngle");
        lAngle.setDirection(Servo.Direction.FORWARD);

        Blocker = hwMap.get(Servo.class,"blocker");
        Blocker.setDirection(Servo.Direction.FORWARD);

        Intake = hwMap.get(DcMotorEx.class,"intake");
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Blocker.setPosition(1);

        AutoState = state.IDLE;
    }

    public void update() {
        switch (AutoState) {
            case IDLE:
                Intake.setPower(0);
                if(ballsRemainig>0){
                    Blocker.setPosition(GATE_CLOSE_ANGLE );
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
                Blocker.setPosition(GATE_OPEN_ANGLE);
                stateTimer.reset();
                AutoState = state.SHOOT;
                break;
            case SHOOT:
                Intake.setPower(1);
                if(stateTimer.seconds()>2){
                    Intake.setPower(0);
                    Blocker.setPosition(GATE_CLOSE_ANGLE );
                    shotsRemaining = 0;
                        stateTimer.reset();
                        AutoState = state.IDLE;
                }
                break;
            case INTAKE:
                if (stateTimer.seconds() > intakeDuration) { // Runs intake for 3 seconds
                    ballsRemainig = 0;
                    Intake.setPower(1);
                    Blocker.setPosition(GATE_CLOSE_ANGLE);
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

    public void setIntakeDuration(double duration){
        intakeDuration = duration;
    }

    public boolean IDLE() {
        return AutoState == state.IDLE;
    }
}
