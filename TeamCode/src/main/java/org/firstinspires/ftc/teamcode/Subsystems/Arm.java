package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm extends SubsystemBase {
    private Motor leftArm, rightArm, leftElevator, rightElevator;
    private HardwareMap hwMap;
    private static final double kP = 0.1;  // Ajuste conforme necessário
    private double targetPosition = 0;  // Posição alvo desejada
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
        targetPosition = position;
    }

    public void updatePosition(){
        double currentLeftPosition = leftElevator.getCurrentPosition();
        double currentRightPosition = rightElevator.getCurrentPosition();

        // Calcula o erro de posição
        double leftError = targetPosition - currentLeftPosition;
        double rightError = targetPosition - currentRightPosition;

        // Aplica o ganho proporcional ao erro para determinar a potência
        double leftPower = kP * leftError;
        double rightPower = kP * rightError;

        leftElevator.set(leftPower);
        rightElevator.set(rightPower);

        Log.d("Arm", String.format("Left Power: %f, Right Power: %f", leftPower, rightPower));

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
