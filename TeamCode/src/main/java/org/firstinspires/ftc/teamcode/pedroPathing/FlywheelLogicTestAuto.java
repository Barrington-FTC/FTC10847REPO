package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.IntakeLogic;
import org.firstinspires.ftc.teamcode.mechanisms.flyWheelLogic;

@Autonomous(name = "Flywheel Logic Test Auto", group = "Autonomous")
@Configurable // Panels
public class FlywheelLogicTestAuto extends OpMode {

    private flyWheelLogic shooter = new flyWheelLogic();

    private IntakeLogic intaker = new IntakeLogic();

    private boolean shotsTriggered = false;

    private PathConstraints constraints = new PathConstraints(1, .1, 1, .5, .5, .3, 1, .7);// so you can use the is busy
    // funtion not my bullshit
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)

    private Timer pathTimer, actionTimer, opmodeTimer;
    private DcMotorEx Turret = null;
    private int Targetpos = 100;
    private DcMotorEx Intake;
    private Servo Blocker;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        // adds tollerances for when a path is considered complete
        follower.setStartingPose(new Pose(48, 135, Math.toRadians(180)));
        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        Intake = hardwareMap.get(DcMotorEx.class, "intake");
        Blocker = hardwareMap.get(Servo.class,"blocker");
        shooter.init(hardwareMap,Blocker,Intake,1650);
        intaker.init(hardwareMap,Blocker,Intake);
        Turret.setDirection(DcMotorSimple.Direction.REVERSE);
        Turret.setTargetPosition(270);
        Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Turret.setPositionPIDFCoefficients(95);
        Turret.setPower(1);
         // Build paths
        Blocker.setPosition(.9);
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        shooter.update();
        intaker.update();
        pathState = autonomousPathUpdate();// Update autonomous state machine

        // makes sure teleop gets position thats stopped on

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.update(telemetry);
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                    setPathState(1);
                break;
            case 1:
                shooter.fireShots(3);
                if(shooter.IDLE() && pathTimer.getElapsedTimeSeconds()>1){
                    shooter.forceCloseGate();
                    if(pathTimer.getElapsedTimeSeconds()>2){
                        Blocker.setPosition(.9);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                Intake.setPower(1);
                if(pathTimer.getElapsedTimeSeconds()>5){
                    Intake.setPower(0);
                    setPathState(3);
                }
                break;
            case 3:
                    setPathState(4);
                break;
            case 4:
                shooter.fireShots(3);
                if(shooter.IDLE() && pathTimer.getElapsedTimeSeconds()>1){
                    shooter.fireShots(0);
                    setPathState(5);
                }
                break;
            case 5:
                    setPathState(6);
                break;
            case 6:
                intaker.intakeBALLZ(1);
                if(pathTimer.getElapsedTimeSeconds()>3){
                    intaker.forceStopIntake();
                    setPathState(7);
                }
                break;
            case 7:
                setPathState(8);
                break;
            case 8:
                shooter.fireShots(3);
                if(shooter.IDLE() && pathTimer.getElapsedTimeSeconds()>1){
                    shooter.forceCloseGate();
                    setPathState(9);
                }
                break;

            case 9:
                    setPathState(10);
                break;
            case 10:
                /*
                savedPosition.setX(follower.getPose().getX());
                savedPosition.sety(follower.getPose().getY());
                savedPosition.seth(follower.getPose().getHeading());
                requestOpModeStop();

                 */
        }
        return pathState;
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    private boolean followerArivved(){
        if((follower.getPose().getX()>follower.getCurrentPath().endPose().getX()-1 && follower.getPose().getX()<follower.getCurrentPath().endPose().getX()+1)&&(follower.getPose().getY()>follower.getCurrentPath().endPose().getY()-1 && follower.getPose().getY()<follower.getCurrentPath().endPose().getY()+1)&&(follower.getVelocity().getMagnitude()<.2)){
            return true;
        }
        else{
            return false;
        }

    }
}
