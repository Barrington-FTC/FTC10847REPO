package org.firstinspires.ftc.teamcode.PracticeOpModes;

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
import org.firstinspires.ftc.teamcode.Services.turretAimingService;


@Config
@TeleOp(name="4w Swerve")
public class FourWheelSWERVEDrive extends LinearOpMode {
    //drive train
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;

    private Servo leftFrontServo = null;
    private Servo leftBackServo = null;
    private Servo rightFrontServo = null;
    private Servo rightBackServo = null;
    private GoBildaPinpointDriver pinpoint = null;
    private double L = 10; //wheelbase (front-back distance)inches
    private double  W = 10; //trackwidth (left-right distance)
    private double R = Math.sqrt(Math.pow(L,2) + Math.pow(W,2));
    private double A;
    private double B;
    private double C;
    private double D;
    private double rightFrontPower;
    private double leftFrontPower;
    private double leftBackPower;
    private double rightBackPower;


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



        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) { // Loop
            //update all sensor variable to make cycle times faster at the start


            // --------------------------- WHEELS --------------------------- //

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;  // forward
            double rot = gamepad1.right_stick_x; // rotation
            A = x - rot * (L / R);
            B = x + rot * (L / R);
            C = y - rot * (W / R);
            D = y + rot * (W / R);
            //this gives 180 to -180 but thats like super ineffient for the servo to move that much
            double frAngle = Math.atan2(B, C);
            if(frAngle<0){
                rightFrontPower = -1*Math.sqrt(B*B + D*D); //change direction
                frAngle = frAngle + Math.toRadians(180);;//re ajust angle to make it 0 to 180
            }
            else{
                rightFrontPower = Math.sqrt(B*B + D*D);
            }
            double flAngle = Math.atan2(B, D);
            if(flAngle<0){
                leftFrontPower = -1*Math.sqrt(B*B + C*C);
                flAngle = flAngle + Math.toRadians(180);
            }
            else{
                leftFrontPower = Math.sqrt(B*B + C*C);
            }
            double blAngle = Math.atan2(A, D);
            if(blAngle<0){
                leftBackPower = -1 * Math.sqrt(A*A + D*D);
                blAngle = blAngle + Math.toRadians(180);
            }
            else{
                leftBackPower = Math.sqrt(A*A + D*D);
            }
            double brAngle = Math.atan2(A, C);
            if(brAngle<0){
                rightBackPower = -1 * Math.sqrt(A*A + C*C);
                brAngle = brAngle + Math.toRadians(180);
            }
            else{
                rightBackPower = Math.sqrt(A*A + C*C);
            }




            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            //set the servos to the correct direction.




            // --------------------------- TELEMETRY --------------------------- //
            // Show the elapsed game time and wheel power.

            telemetry.update();
        }
    }

    private double calculate(double x){
        return 3.5345*x+1326.24468;
    }

    // Dedicated method for the PID loop
    private void setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }
}
