package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SharedMotorAndServos {
    public Servo Blocker;
    public DcMotorEx Intake;

    public void init(HardwareMap hw){
        Blocker = hw.get(Servo.class,"blocker");
        Blocker.setDirection(Servo.Direction.FORWARD);
        Intake = hw.get(DcMotorEx.class,"intake");
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void setBlockerPosition(double pos){
        Blocker.setPosition(pos);
    }
    public void setIntakePower(double pow){
        Intake.setPower(pow);
    }
    public double getBlockerPosition(){
        return Blocker.getPosition();
    }

}
