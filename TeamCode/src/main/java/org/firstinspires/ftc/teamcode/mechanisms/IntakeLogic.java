package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeLogic {

    private DcMotorEx intakeMotor = null;
    private ElapsedTime stateTimer = new ElapsedTime();

    private enum IntakeState {
        IDLE,
        INTAKING
    }

    private IntakeState intakeState;

    private Servo blocker = null;
    private int ballsToTake = 0;

    public void init(HardwareMap hwMap, Servo Blocker,DcMotorEx Intake) {
        intakeMotor = Intake;
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blocker = Blocker;
        intakeState = IntakeState.IDLE;
    }

    public void update() {
        switch (intakeState) {
            case IDLE:
                if (ballsToTake > 0) {
                    blocker.setPosition(.9);
                    intakeMotor.setPower(1);
                    stateTimer.reset();
                    intakeState = IntakeState.INTAKING;
                }
                break;
            case INTAKING:
                if (stateTimer.seconds() > 5) { // Runs intake for 3 seconds
                    ballsToTake = 0;
                    intakeMotor.setPower(0);
                    intakeState = IntakeState.IDLE;
                }
                break;

        }
    }

    /**
     * Triggers the intake to run for a fixed duration.
     * @param ballCount The number of balls to attempt to intake.
     */
    public void intakeBALLZ(int ballCount) {
        if (intakeState == IntakeState.IDLE) {
            this.ballsToTake = ballCount;
        }
    }

    public boolean isBusy() {
        return intakeState != IntakeState.IDLE;
    }
    public void forceStopIntake(){
        intakeMotor.setPower(0);
    }

    public void setIntakePower(double pow){
        intakeMotor.setPower(pow);
    }
}
