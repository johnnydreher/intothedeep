package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm extends SubsystemBase {
    private Motor leftArm, rightArm, leftElevator, rightElevator;
    private HardwareMap hwMap;
    public Arm() {
    }
    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;
        leftArm = new Motor(hwMap,"leftArm");
        rightArm = new Motor(hwMap,"rightArm");

        leftElevator = new Motor(hwMap,"leftElevator");
        rightElevator = new Motor(hwMap,"rightElevator");
        //right.setInverted(true);
        leftArm.setRunMode(Motor.RunMode.RawPower);
        rightArm.setRunMode(Motor.RunMode.RawPower);

        leftElevator.setRunMode(Motor.RunMode.PositionControl);
        rightElevator.setRunMode(Motor.RunMode.PositionControl);
    }

    public void setPower(double power){
        leftArm.set(power);
        rightArm.set(power);
        Log.d("Arm",String.format("Out: %f",power));

    }

    public void setPosition(double position){
        leftElevator.set(position);
        rightElevator.set(position);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
