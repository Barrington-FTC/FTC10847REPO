package org.firstinspires.ftc.teamcode.RoboticsClassCode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Services.PIDService;
@Autonomous(name = "Sid and will auto", group = "Autonomous")
@Configurable // Panels
public class SidandWillAuto extends LinearOpMode {
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;

    private PIDService PIDservice = new PIDService();


    //turret
    private DcMotorEx intake = null;
    private DcMotorEx transfer = null;
    private DcMotorEx flyWheel = null;

    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBackDrive");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        //intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        //turret
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        flyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        flyWheel.setPower(.92);
        driveBack(.7);
        sleep(750);
        brake();
        sleep(500);
        driveForward(.7);
        intake.setPower(1);
        transfer.setPower(1);
        sleep(500);
        driveBack(1);
        sleep(100);
        driveForward(1);
        sleep(250);
        brake();
        sleep(1000);
        strafeLeft(.7);
        sleep(500);
        brake();
        intake.setPower(0);
        transfer.setPower(0);
        sleep(500);
        requestOpModeStop();


    }
    public void driveBack(double power){
        leftFrontDrive.setPower(-power);
        leftBackDrive.setPower(-power);
        rightFrontDrive.setPower(-power);
        rightBackDrive.setPower(-power);
    }
    public void driveForward(double power){
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
    }
    public void strafeLeft(double power){
        leftFrontDrive.setPower(-power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(-power);
    }
    public void strafeRight(double power){
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(-power);
        rightFrontDrive.setPower(-power);
        rightBackDrive.setPower(power);
    }
    public void brake(){
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
