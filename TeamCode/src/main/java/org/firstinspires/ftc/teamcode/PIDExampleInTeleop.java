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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@Config
@TeleOp(name="PIDExampleInTeleop")
public class PIDExampleInTeleop extends LinearOpMode {
    // when coding a robot with multiple systems it is very important to group you variables with things that are alike.

    //as of year 2025-2026 this is the proper way to declare a variable use DcMotorEx class as it has the most amount of features and there are no downsides
    private DcMotorEx leftFrontDrive = null;//when naming variables pick names that explain what it does
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;

    private DcMotorEx RPMPIDMotor = null;//will be used to showcase how to use a pidf for a motors velocity

    private DcMotorEx PositionPIDMotor = null;//will be used to showcase the use of a pidf for a motors position.



    //below we have pinpoint declaration this will should be the same across all coding years
    private GoBildaPinpointDriver pinpoint = null;

    //these are your pinpoint offsets they are used to make sure the pinpoint can accuratly tell where the center of your robot is find the values in your cad model
    //DO NOT FIND THESE VALUES BY MEASURING TAPE
    //for further guidence go to this website you will likley be using the goBuilda pinpoint look at the diagram halfway down the page: https://pedropathing.com/docs/pathing/tuning/localization/pinpoint
    private static final double yOffset = -129.3;
    private static final double xOffset = 100;


    //when grouping vars label what they are use for
    // Odometry constants
    Pose2D currentPose = new Pose2D(DistanceUnit.INCH,48, 8.0826771654, AngleUnit.DEGREES,90);// GoBuilda pinpoints and all odomitry pods use dead reckoning meaning you need to know where you start on the field.
    private double xPosition;//although you can call the pinpoints position through method calls it best to store the most recent value in a readable variables making it easy understand when debuging,
    private double yPosition;
    private double heading;
    private double distanceToTarget;
    private double xVelocity;
    private double yVelocity;
    private double netV;

    //if you want to track a target use the pedro pathing cordnate system as it is better than the First cord system and will align with the pedro pathing can be found here: https://pedropathing.com/docs/pathing/reference/coordinates
    private double targetx; // some games in the future may need you to know the distance to a specific target store these in variables names not as numbers in the code to make it readable.
    private double targety;




    @Override
    public void runOpMode() {
        //drive train
        //this initalise your motor object and will use the text in quotes as your name in the driver station configuration file
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBackDrive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RPMPIDMotor = hardwareMap.get(DcMotorEx.class,"RPMPIDMotor");
        PositionPIDMotor = hardwareMap.get(DcMotorEx.class,"PositionPIDMotor");

        RPMPIDMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        PositionPIDMotor.setDirection(DcMotorSimple.Direction.FORWARD);



        //init for telling the motors on how they should use there encoders if you do not set the run mode to run using encoder or run to encoder its encoder will not report its position back.
        RPMPIDMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PositionPIDMotor.setTargetPosition(0);//required before setting a motors run mode to a position.
        PositionPIDMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);//allows for the built in function setTargetPosition() to be called

        PIDFCoefficients rpmPIDFCOEFF = new PIDFCoefficients(0,0,0,0);//watch a tutorial on tuning a pid and what p i d and f mean
        PIDFCoefficients posPIDFCOEFF = new PIDFCoefficients(0,0,0,0);
        //sets each motor to use the PIDF coefficients that you assigned.
        RPMPIDMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,rpmPIDFCOEFF);
        PositionPIDMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,posPIDFCOEFF);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.MM);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        pinpoint.recalibrateIMU();


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        pinpoint.setPosition(currentPose);
        while (opModeIsActive()) { // Loop
            //update all sensor variable to make cycle times faster at the start
            pinpoint.update();
            xPosition = pinpoint.getPosX(DistanceUnit.INCH);
            yPosition = pinpoint.getPosY(DistanceUnit.INCH);
            heading = pinpoint.getHeading(AngleUnit.RADIANS);
            distanceToTarget = Math.sqrt(Math.pow(xPosition - targetx, 2) + Math.pow(yPosition - targety, 2));
            xVelocity = pinpoint.getVelX(DistanceUnit.INCH);
            yVelocity = pinpoint.getVelY(DistanceUnit.INCH);
            netV = Math.sqrt(Math.pow(xVelocity, 2) + Math.pow(yVelocity, 2));


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


            //used to make the pid correct the motors position/velocity
            //0 is a placeholder
            RPMPIDMotor.setVelocity(0);
            PositionPIDMotor.setTargetPosition(0);

            // --------------------------- TELEMETRY --------------------------- //
            // Show the elapsed game time and wheel power.
            telemetry.addData("x", pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("y", pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("heading (deg)", pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }

    // Dedicated method for the PID loop
    private void setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }
}
