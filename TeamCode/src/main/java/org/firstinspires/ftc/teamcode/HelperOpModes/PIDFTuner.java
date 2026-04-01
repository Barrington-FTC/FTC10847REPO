package org.firstinspires.ftc.teamcode.HelperOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Services.PIDFController;
import org.firstinspires.ftc.teamcode.Services.PIDService;


@Config
@TeleOp(name="PIDTuner")
public class PIDFTuner extends LinearOpMode {

    private ElapsedTime elapsedTime = new ElapsedTime();

    private PIDService PIDservice = new PIDService();

    private double flyWheelTargetRPM = 2000;

    private Servo Blocker = null;
    private DcMotorEx intake = null;

    private DcMotorEx flyWheelR = null;
    private DcMotorEx flyWheelL = null;
    private double amount = 1;

    private double kf = 0;

    private double kp = 0;
    private boolean toggle = false;
    private PIDFController pidfController = new PIDFController(0,0,0,0);
    @Override
    public void runOpMode() {
        //drive train
        Blocker = hardwareMap.get(Servo.class,"blocker");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        flyWheelR = hardwareMap.get(DcMotorEx.class, "flyWheelR");
        flyWheelL = hardwareMap.get(DcMotorEx.class, "flyWheelL");

        flyWheelR.setDirection(DcMotorEx.Direction.FORWARD);
        flyWheelR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheelR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flyWheelL.setDirection(DcMotorEx.Direction.REVERSE);
        flyWheelL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheelL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Blocker.setPosition(.9);
        waitForStart();
        pidfController.reset();
        elapsedTime.startTime();
        while (opModeIsActive()) { // Loop
            if(gamepad1.dpadUpWasPressed()){
                kp += amount;
                pidfController.setKp(kp);
            }
            if(gamepad1.dpadDownWasPressed()){
                kp -= amount;
                pidfController.setKp(kp);
            }
            if(gamepad1.dpadRightWasPressed()){
                kf += amount;
                pidfController.setKf(kf);
            }
            if(gamepad1.dpadLeftWasPressed()){
                kf -= amount;
                pidfController.setKf(kf);
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
            if(gamepad1.right_trigger>.1){
                intake.setPower(1);
            }
            else{
                intake.setPower(0);
            }
            if(gamepad1.right_stick_button){
                if(toggle){
                    Blocker.setPosition(1);
                    toggle = false;
                }
                else{
                    Blocker.setPosition(.9);
                    toggle = true;
                }
            }
            pidfController.setTargetVelocity(flyWheelTargetRPM);

            pidfController.calculate_velocity(flyWheelL.getCurrentPosition(),elapsedTime.seconds());
            flyWheelL.setPower(pidfController.calculate(flyWheelL.getVelocity()));
            flyWheelR.setPower(pidfController.calculate(flyWheelL.getVelocity()));




            // --------------------------- TELEMETRY --------------------------- //
            telemetry.addData("kp", kp);
            telemetry.addData("kf", kf);
            telemetry.addData("amount", amount);//distanceToTarget
            telemetry.addData("Flywheel Target Velocity", flyWheelTargetRPM);//distanceToTarget
            telemetry.addData("Flywheel L Velocity", pidfController.getVelocity());
            telemetry.addData("Flywheel R Velocity", pidfController.getVelocity());
            telemetry.addData("Flywheel Target Velocity", flyWheelTargetRPM);//distanceToTarget
            telemetry.addData("pidf returned power",pidfController.calculate(flyWheelL.getVelocity()));
            telemetry.update();
        }
    }
}
