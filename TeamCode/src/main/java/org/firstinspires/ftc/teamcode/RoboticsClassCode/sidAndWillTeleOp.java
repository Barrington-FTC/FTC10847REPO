package org.firstinspires.ftc.teamcode.RoboticsClassCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Services.PIDFController;
import org.firstinspires.ftc.teamcode.Services.PIDService;

import java.util.List;


@Config
@TeleOp(name="Sid and will")
public class sidAndWillTeleOp extends LinearOpMode {
    //drive train
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;

    private PIDService PIDservice = new PIDService();


    //turret
    private DcMotorEx transfer = null;
    private DcMotorEx flyWheel = null;
    //private Limelight3A limelight;
    //Intake
    private DcMotorEx intake = null;



    private PIDFController pidfController = new PIDFController(PIDservice.getFinalKP(),0,0,PIDservice.getFinalKF());
    private int targetFlywheelVelocity = 2500;

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

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();
        pidfController.reset();
        while (opModeIsActive()) { // Loop
            //this is to reduce time complexity reads all sensor values at start of loop then stores values in cache for fast retreval
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            flyWheel.setPower(.75);


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

            if (gamepad1.right_trigger > .01) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }
            if(gamepad1.left_trigger> 0.01){
                transfer.setPower(1);
            }
            else{
                transfer.setPower(0);
            }
            if(gamepad1.rightBumperWasPressed()){
                targetFlywheelVelocity +=50;
            }
            if(gamepad1.leftBumperWasPressed()){
                targetFlywheelVelocity -=50;
            }
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
