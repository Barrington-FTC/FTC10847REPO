package org.firstinspires.ftc.teamcode.HelperOpModes;

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
import org.firstinspires.ftc.teamcode.Services.PIDService;


@Config
@TeleOp(name="PIDTuner")
public class PIDTuner extends LinearOpMode {

    private PIDService PIDservice = new PIDService();

    private double flyWheelTargetRPM = 2000;

    private DcMotorEx flyWheelR = null;
    private DcMotorEx flyWheelL = null;
    private double amount = 1;
    private int vfOffset = 0;

    private double kf = 0;

    private double kp = 0;

    @Override
    public void runOpMode() {
        //drive train


        flyWheelR = hardwareMap.get(DcMotorEx.class, "flyWheelR");
        flyWheelL = hardwareMap.get(DcMotorEx.class, "flyWheelL");

        flyWheelR.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheelL.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheelL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients Coef = new PIDFCoefficients(kp,0,0,kf);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        while (opModeIsActive()) { // Loop
            if(gamepad1.dpadUpWasPressed()){
                kp += amount;
            }
            if(gamepad1.dpadDownWasPressed()){
                kp -= amount;
            }
            if(gamepad1.dpadRightWasPressed()){
                kf += amount;
            }
            if(gamepad1.dpadLeftWasPressed()){
                kf -= amount;
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
            Coef = new PIDFCoefficients(kp,0,0,kf);
            flyWheelR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Coef);
            flyWheelL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Coef);
            flyWheelR.setVelocity(flyWheelTargetRPM);
            flyWheelL.setVelocity(flyWheelTargetRPM);




            // --------------------------- TELEMETRY --------------------------- //
            telemetry.addData("kp", kp);
            telemetry.addData("kf", kf);
            telemetry.addData("amount", amount);//distanceToTarget
            telemetry.addData("Flywheel Target Velocity", flyWheelTargetRPM);//distanceToTarget
            telemetry.addData("Flywheel L Velocity", flyWheelL.getVelocity());
            telemetry.addData("Flywheel R Velocity", flyWheelR.getVelocity());
            telemetry.addData("Flywheel Target Velocity", flyWheelTargetRPM);//distanceToTarget
            telemetry.update();
        }
    }
}
