package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
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
import org.firstinspires.ftc.teamcode.Services.savedPositionService;

@Autonomous(name = "blue close auto", group = "Autonomous")
@Configurable // Panels
public class BCloseAuto extends OpMode {

    private flyWheelLogic shooter = new flyWheelLogic();

    private IntakeLogic intaker = new IntakeLogic();

    private boolean shotsTriggered = false;

    private PathConstraints constraints = new PathConstraints(1, .1, 1, .5, .5, .3, 1, .7);// so you can use the is busy
    // funtion not my bullshit
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private Timer pathTimer, actionTimer, opmodeTimer;
    private static DcMotorEx Turret = null;
    public Servo Blocker = null;

    private int Targetpos = 220;
    public static int savedTurretPos;
    private DcMotorEx Intake;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        // adds tollerances for when a path is considered complete
        follower.setStartingPose(new Pose(36, 135, Math.toRadians(180)));
        savedPositionService.setX(follower.getPose().getX());
        savedPositionService.sety(follower.getPose().getY());
        savedPositionService.seth(follower.getPose().getHeading());
        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        Blocker = hardwareMap.get(Servo.class,"blocker");
        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        Blocker = hardwareMap.get(Servo.class,"blocker");
        Intake = hardwareMap.get(DcMotorEx.class,"Intake");
        intaker.init(hardwareMap,Blocker,Intake);
        shooter.init(hardwareMap,Blocker,Intake,1650);
        Turret.setDirection(DcMotorSimple.Direction.REVERSE);
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setTargetPosition(100);
        Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Turret.setPositionPIDFCoefficients(100);
        Turret.setPower(1);
        paths = new Paths(follower); // Build paths
        Blocker.setPosition(.9);
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        intaker.update();
        shooter.update();
        pathState = autonomousPathUpdate();// Update autonomous state machine
        Turret.setTargetPosition(Targetpos);
        savedTurretPos = Turret.getCurrentPosition();


        // makes sure teleop gets position thats stopped on
        savedPositionService.setX(follower.getPose().getX());
        savedPositionService.sety(follower.getPose().getY());
        savedPositionService.seth(follower.getPose().getHeading());

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public static PathChain Path1;
        public static PathChain Path2;
        public static PathChain Path3;
        public static PathChain Path4;
        public static PathChain Path5;
        public static PathChain Path6;
        public static PathChain Path7;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 128.000),

                                    new Pose(60.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.000, 84.000),

                                    new Pose(18.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.000, 84.000),

                                    new Pose(60.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.000, 84.000),

                                    new Pose(55.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.000, 60.000),

                                    new Pose(10.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10.000, 60.000),

                                    new Pose(60.000, 85.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.000, 85.000),

                                    new Pose(60.000, 120.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();
        }
    }

    public int autonomousPathUpdate() {
        Turret.setTargetPosition(Targetpos);
        switch (pathState) {
            case 0:
                follower.followPath(Paths.Path1);
                if(followerArivved()){
                    setPathState(1);
                }
                break;
            case 1:
                shooter.fireShots(3);
                if(shooter.IDLE() && pathTimer.getElapsedTimeSeconds()>1){
                    if(pathTimer.getElapsedTimeSeconds()>2){
                        Blocker.setPosition(.9);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                Intake.setPower(1);
                follower.followPath(Paths.Path2);
                if(followerArivved()){
                    Intake.setPower(0);
                    setPathState(3);
                }
                break;

            case 3:
                follower.followPath(Paths.Path3);
                if(followerArivved()){
                    setPathState(4);
                }
                break;
            case 4:
                shooter.fireShots(3);
                if(shooter.IDLE() && pathTimer.getElapsedTimeSeconds()>1){
                    if(pathTimer.getElapsedTimeSeconds()>2){
                        Blocker.setPosition(.9);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                follower.followPath(Paths.Path4);
                if(followerArivved()){
                    setPathState(6);
                }
                break;
            case 6:
                Intake.setPower(1);
                follower.followPath(Paths.Path2);
                if(followerArivved()){
                    Intake.setPower(0);
                    setPathState(7);
                }
                break;
            case 7:
                follower.followPath(Paths.Path6);
                if(followerArivved()){
                    setPathState(8);
                }
                break;
            case 8:
                shooter.fireShots(3);
                if(shooter.IDLE() && pathTimer.getElapsedTimeSeconds()>1){
                    if(pathTimer.getElapsedTimeSeconds()>2){
                        Blocker.setPosition(.9);
                        setPathState(9);
                    }
                }
                break;

            case 9:
                follower.followPath(Paths.Path7);
                if(followerArivved()){
                    setPathState(10);
                }
                break;
            case 10:
                savedPositionService.setX(follower.getPose().getX());
                savedPositionService.sety(follower.getPose().getY());
                savedPositionService.seth(follower.getPose().getHeading());
                requestOpModeStop();
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
    public static DcMotorEx getTurret(){
        return Turret;
    }
    public static int getLastTurretPos(){
        return savedTurretPos;
    }
}
