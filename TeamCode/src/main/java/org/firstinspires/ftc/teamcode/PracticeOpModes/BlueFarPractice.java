package org.firstinspires.ftc.teamcode.PracticeOpModes;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Services.PIDController;
import org.firstinspires.ftc.teamcode.Services.PIDFController;
import org.firstinspires.ftc.teamcode.Services.PIDService;
import org.firstinspires.ftc.teamcode.Services.flywheelService;
import org.firstinspires.ftc.teamcode.Services.turretAimingService;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;



@Config
@TeleOp(name="Blue Far Practice")
public class BlueFarPractice extends LinearOpMode {
    private ElapsedTime elapsedTime = new ElapsedTime();
    //drive train
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private GoBildaPinpointDriver pinpoint = null;

    private PIDService PIDservice = new PIDService();


    // Odometry constants
    Pose2D currentPose = new Pose2D(DistanceUnit.INCH,48, 8.0826771654, AngleUnit.DEGREES,90);//used to save position after autonomous
    //Pose2D currentPose = new Pose2D(DistanceUnit.INCH,savedPosition.getX(), savedPosition.getX(),AngleUnit.DEGREES, savedPosition.getHeading());used to save position after autonomous
    //offsets
    private static final double yOffset = -129.3;
    private static final double xOffset = 100;

    //turret
    private DcMotorEx flyWheelR = null;
    private DcMotorEx flyWheelL = null;
    private DcMotorEx Turret = null;
    private Servo lAngle = null;
    //private Limelight3A limelight;
    //Intake
    private DcMotorEx intake = null;
    private Servo blocker = null;

    //variables
    private double xPosition = 0;

    private double xVelocity = 0; //Velocity in x direction
    private double yPosition = 0;
    private double yVelocity = 0; //Velocity in y direction

    private double heading = 0;

    private double distanceToTarget = 0;

    //using pedro pathing cordnate system
    private double targetx = 0;//location of field//red is 3.556m(center of target 4 inches away from wall) blue is 0.1016m(center of target + 4 inches from the wall)
    private double targety = 144;//location on feild always 3.4544m

    private double maxMotorPower = 1;

    private double zeroMotorPower = 0;

    private double halfMotorPower = 0.5;

    private double intakeIdlePower = 0.65;

    private double servoCloseAngle = .92;

    private double servoOpenAngle = 1;

    private double controllerDeadzone = .01;

    private boolean blockerToggle = true;

    private turretAimingService turretAimingService = new turretAimingService();

    private flywheelService FlywheelService = new flywheelService();

    private PIDFController pidfController = new PIDFController(PIDservice.getFinalKP(),0,0,PIDservice.getFinalKF());

    private PIDController pidController = new PIDController(1,0.06,0.05);//placholder value need to tune turret
    private boolean intakeToggle = false;


    @Override
    public void runOpMode() {
        //drive train
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBackDrive");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(xOffset, yOffset,DistanceUnit.MM);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        pinpoint.recalibrateIMU();


        //intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        //turret
        flyWheelR = hardwareMap.get(DcMotorEx.class, "flyWheelR");
        flyWheelL = hardwareMap.get(DcMotorEx.class, "flyWheelL");
        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        lAngle = hardwareMap.get(Servo.class, "lAngle");

        blocker = hardwareMap.get(Servo.class, "blocker");
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        flyWheelR.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheelR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheelR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyWheelL.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheelL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheelL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        Turret.setDirection(DcMotorSimple.Direction.FORWARD);
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //services
        turretAimingService.initTurretAiming(targetx,targety);


        blocker.setPosition(servoCloseAngle);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();
        pinpoint.setPosition(currentPose);
        pidfController.reset();
        pidController.reset();
        elapsedTime.startTime();
        while (opModeIsActive()) { // Loop
            //this is to reduce time complexity reads all sensor values at start of loop then stores values in cache for fast retreval
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            // 2. Update sensors immediately
            pinpoint.update();
            xPosition = pinpoint.getPosX(DistanceUnit.INCH);
            yPosition = pinpoint.getPosY(DistanceUnit.INCH);
            heading = pinpoint.getHeading(AngleUnit.RADIANS);

            // 3. Optimize Math (x*x instead of Math.pow) becuase it has a lower time complexity
            double dx = xPosition - targetx;
            double dy = yPosition - targety;
            distanceToTarget = Math.sqrt(dx * dx + dy * dy);

            xVelocity = pinpoint.getVelX(DistanceUnit.INCH);
            yVelocity = pinpoint.getVelY(DistanceUnit.INCH);


            double flyhweelVelocity = flyWheelL.getVelocity();
            pidfController.setTargetVelocity(calculate(distanceToTarget));
            double flywheelPower = pidfController.calculate(flyhweelVelocity);
            flyWheelL.setPower(flywheelPower);
            flyWheelR.setPower(flywheelPower);

            // 5. Update Turret
            pidController.setTargetPosition(turretAimingService.aimTurret(xPosition, yPosition, heading));
            double turretPower = pidController.calculate(Turret.getCurrentPosition());
            Turret.setPower(turretPower);

            // 5. Update Turret



            // --------------------------- WHEELS --------------------------- //
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = Math.pow(-gamepad1.left_stick_y, 3);  // Note: pushing stick forward gives negative value
            double lateral = Math.pow(gamepad1.left_stick_x, 3);
            double yaw = Math.pow(gamepad1.right_stick_x, 3);
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > maxMotorPower) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
            if(gamepad1.left_trigger > controllerDeadzone){
                leftFrontPower *= halfMotorPower;
                rightFrontPower *= halfMotorPower;
                leftBackPower *= halfMotorPower;
                rightBackPower *= halfMotorPower;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            if(gamepad1.right_trigger > controllerDeadzone){
                intake.setPower(maxMotorPower);
            }
            else {
                intake.setPower(zeroMotorPower);
            }


            if(gamepad1.xWasPressed()){
                pinpoint.setHeading(90,AngleUnit.DEGREES);
            }
            if(gamepad1.aWasPressed()){
                if(blockerToggle){
                    blockerToggle = false;
                    blocker.setPosition(servoOpenAngle);
                }
                else{
                    blockerToggle = true;
                    blocker.setPosition(servoCloseAngle);
                }
            }
            if(gamepad1.yWasPressed()){
                if(intakeToggle){
                    intakeToggle = false;
                }
                else{
                    intakeToggle = true;
                }
            }

        }

    }
    private double calculate(double x){
        return 4.5*x+1190.24468;
    }
    private void setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }
}
