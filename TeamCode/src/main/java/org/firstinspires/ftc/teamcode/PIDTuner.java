package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.teamcode.Services.PIDService;


@Config
@TeleOp(name="PIDTuner")
public class PIDTuner extends LinearOpMode {
    //drive train
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private GoBildaPinpointDriver pinpoint = null;

    private PIDService PIDservice = new PIDService();


    // Odometry constants
    Pose2D currentPose = new Pose2D(DistanceUnit.INCH,48, 8.0826771654, AngleUnit.DEGREES,90);//used to save position after autonomous

    //offsets
    private static final double yOffset = -129.3;
    private static final double xOffset = 100;

    private static final double TURRET_MOTOR_TICKS_PER_REVOLUTION = 145.1;
    // This is the gear ratio between the motor and the turret.
    private static final double TURRET_GEAR_RATIO = 5.0; // Change this to match your gear ratio
    private static final double TURRET_TICKS_PER_RADIAN = (TURRET_MOTOR_TICKS_PER_REVOLUTION * TURRET_GEAR_RATIO) / (2 * Math.PI);

    private double flyWheelTargetRPM = 2000;

    private double kp = 5.85;

    private double kf = 0.6;

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
    private double x = 0;

    private double xV = 0; //Velocity in x direction
    private double y = 0;
    private double yV = 0; //Velocity in y direction

    private double netV = 0; //net Vector of velocity
    private double heading = 0;

    private double hV = 0; //Velocity in heading direction
    private double distanceToTarget = 0;
    private double tx = 0;

    //using pedro pathing cordnate system
    private double targetx = 4;//location of field//red is 3.556m(center of target 4 inches away from wall) blue is 0.1016m(center of target + 4 inches from the wall)
    private double targety = 144;//location on feild always 3.4544m
    private double relTargetangle = 0;// angle from from of robot to target calculate later relative to the bot cordnate system
    private double targetangle = 0; // angle from from of robot to target calculate later relative to the field cordnate system
    private int turretTargetPosition;
    private int turretmaxr = 0;
    private int turretmaxl = 370;

    private boolean toggle = true;
    private double amount = 1;
    private int vfOffset = 0;

    @Override
    public void runOpMode() {
        //drive train

        /*
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

         */
        flyWheelR = hardwareMap.get(DcMotorEx.class, "flyWheelR");
        flyWheelL = hardwareMap.get(DcMotorEx.class, "flyWheelL");
        /*
        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        lAngle = hardwareMap.get(Servo.class, "lAngle");

        blocker = hardwareMap.get(Servo.class, "blocker");
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");

         */
        flyWheelR.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheelL.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheelL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

/*
        Turret.setDirection(DcMotorSimple.Direction.REVERSE);
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Turret.setTargetPosition(0);
        Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Turret.setPositionPIDFCoefficients(100);
        Turret.setPower(1);


 */

        flyWheelR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDservice.getFlywheelCoefficents());
        flyWheelL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDservice.getFlywheelCoefficents());


        //blocker.setPosition(.90);

        //lAngle.setPosition(1);

       // Thread operationsThread = new Thread(this::operations);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        //operationsThread.start();
        //pinpoint.setPosition(currentPose);
        while (opModeIsActive()) { // Loop
            //update all sensor variable to make cycle times faster at the start
            /*
            pinpoint.update();

            x = pinpoint.getPosX(DistanceUnit.INCH);
            y = pinpoint.getPosY(DistanceUnit.INCH);
            heading = pinpoint.getHeading(AngleUnit.RADIANS);
            distanceToTarget = Math.sqrt(Math.pow(x - targetx, 2) + Math.pow(y - targety, 2));
            xV = pinpoint.getVelX(DistanceUnit.INCH);
            yV = pinpoint.getVelY(DistanceUnit.INCH);
            netV = Math.sqrt(Math.pow(xV, 2) + Math.pow(yV, 2));
            targetangle = Math.atan2(targety - y, targetx - x);
            relTargetangle = targetangle - heading;
            relTargetangle = Math.atan2(Math.sin(relTargetangle), Math.cos(relTargetangle));

// Shift so forward = pi/2
            double turretAngle = relTargetangle + (Math.PI / 2.0);

// Clamp to turret range
            turretAngle = Math.max(0.0, Math.min(Math.PI, turretAngle));

// Convert to ticks
            turretTargetPosition = (int)(turretAngle * TURRET_TICKS_PER_RADIAN);
            // Clamp the target position to within the physical limits of the turret
            turretTargetPosition = Math.max(turretmaxr,
                    Math.min(turretmaxl, turretTargetPosition));
            vF = calculate(distanceToTarget);


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
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            if(gamepad1.right_trigger>.01){
                if(!toggle){
                    intake.setPower(.8);
                }
                else{
                    intake.setPower(1);
                }
            }
            else {
                intake.setPower(0);
            }
            if(gamepad1.aWasPressed()){
                if(toggle){
                    toggle = false;
                    blocker.setPosition(1);
                }
                else{
                    toggle = true;
                    blocker.setPosition(.90);
                }
            }

            if(gamepad1.yWasPressed()){
                lAngle.setPosition(1);
            }
            if(gamepad1.bWasPressed()){
                lAngle.setPosition(0);
            }

            if(gamepad1.xWasPressed()){
                pinpoint.setHeading(90,AngleUnit.DEGREES);
            }
            if(gamepad1.leftBumperWasPressed()){
                vfOffset -=10;
            }
            if ((gamepad1.rightBumperWasPressed())){
                vfOffset +=10;
            }


             */
            if(gamepad1.dpadUpWasPressed()){
                PIDservice.setFlyhweelKP(PIDservice.getFlyhweelKP() + amount);
            }
            if(gamepad1.dpadDownWasPressed()){
                PIDservice.setFlyhweelKP(PIDservice.getFlyhweelKP() - amount);
            }
            if(gamepad1.dpadRightWasPressed()){
                PIDservice.setFlyhweelKF(PIDservice.getFlywheeKF() + amount);
            }
            if(gamepad1.dpadLeftWasPressed()){
                PIDservice.setFlyhweelKF(PIDservice.getFlywheeKF() - amount);
            }
            if(gamepad1.rightBumperWasPressed()){
                flyWheelTargetRPM+=amount;
            }
            if(gamepad1.leftBumperWasPressed()){
                flyWheelTargetRPM-=amount;
            }
            if(gamepad1.bWasPressed()){
                amount*=10;
            }
            if(gamepad1.aWasPressed()){
                amount/=10;
            }
            flyWheelR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDservice.getFlywheelCoefficents());
            flyWheelL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDservice.getFlywheelCoefficents());
            flyWheelR.setVelocity(flyWheelTargetRPM + vfOffset);
            flyWheelL.setVelocity(flyWheelTargetRPM + vfOffset);
            //Turret.setTargetPosition(turretTargetPosition);



            // --------------------------- TELEMETRY --------------------------- //
            /*
            // Show the elapsed game time and wheel power.
            telemetry.addData("x", pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("y", pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("heading (deg)", pinpoint.getHeading(AngleUnit.DEGREES));

             */
            telemetry.addData("kp", PIDservice.getFlyhweelKP());
            telemetry.addData("kf", PIDservice.getFlywheeKF());
            telemetry.addData("amount", amount);//distanceToTarget
            telemetry.addData("Flywheel Target Velocity", flyWheelTargetRPM);//distanceToTarget
            telemetry.addData("Flywheel L Velocity", flyWheelL.getVelocity());
            telemetry.addData("Flywheel R Velocity", flyWheelR.getVelocity());
            telemetry.addData("Flywheel Target Velocity", flyWheelTargetRPM);//distanceToTarget
            /*
            telemetry.addData("distanceToTarget", distanceToTarget);
            telemetry.addData("Rotation Position", Turret.getCurrentPosition());
            telemetry.addData("Rotation Target Position", turretTargetPosition);
            telemetry.addData("Turret power", Turret.getPower());
            telemetry.addData("Blocker Positon", blocker.getPosition());
            telemetry.addData("lAngel Positon", lAngle.getPosition());

             */
            telemetry.update();
        }
    }

    private double calculate(double x){
        return .5*x+1933.88716;
    }
    private void operations(){
        Turret.setTargetPosition(turretTargetPosition);
        sleep(20);
    }

    // Dedicated method for the PID loop
    private void setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }
}
