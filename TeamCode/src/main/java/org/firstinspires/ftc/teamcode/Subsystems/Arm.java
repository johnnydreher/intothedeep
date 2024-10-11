package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm extends SubsystemBase {
    private Motor left, right;
    private HardwareMap hwMap;
    public Arm() {
    }
    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;
        left = new Motor(hwMap,"leftArm");
        right = new Motor(hwMap,"rightArm");
        //right.setInverted(true);
        left.setRunMode(Motor.RunMode.RawPower);
        right.setRunMode(Motor.RunMode.RawPower);
    }
    public void setPower(double power){
        left.set(power);
        right.set(power);
        Log.d("Arm",String.format("Out: %f",power));

    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
