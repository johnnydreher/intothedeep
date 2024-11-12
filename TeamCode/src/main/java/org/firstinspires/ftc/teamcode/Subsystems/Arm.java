package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.PIDControl;
import org.firstinspires.ftc.teamcode.Utils.PIDFControl;

@Config
public class Arm extends SubsystemBase {
    private MotorEx leftArm, rightArm, leftElevator, rightElevator;
    private HardwareMap hwMap;
    public static double kPArm = 0.001, kIArm = 0.0001, kDArm = 0.0001, kFArm = 0.15;
    public static double kPElevator = 0.002, kIElevator = 0, kDElevator = 0;
    private final PIDFControl pidfArm;
    private final PIDControl pidElevator;
    public static double maxArmPosition = 1200;
    public static double middleArmDown = 100;
    public static double maxElevatorPosition = 9500;
    public static double middleElevatorDown = 4500;
    private boolean down = false;
    public Arm() {
        pidfArm = new PIDFControl(0,kPArm,kIArm,kDArm,kFArm);
        pidElevator = new PIDControl(0,0.1,0,0);
    }
    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;
        leftArm = new MotorEx(hwMap,"leftArm");
        rightArm = new MotorEx(hwMap,"rightArm");
        leftElevator = new MotorEx(hwMap,"leftElevator");
        rightElevator = new MotorEx(hwMap,"rightElevator");
        leftArm.setInverted(true);
        rightArm.setInverted(true);
        leftElevator.setInverted(true);
        rightElevator.setInverted(true);
        leftElevator.setRunMode(Motor.RunMode.RawPower);
        rightElevator.setRunMode(Motor.RunMode.RawPower);


        leftElevator.setRunMode(Motor.RunMode.RawPower);
        rightElevator.setRunMode(Motor.RunMode.RawPower);

        leftArm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        resetEncodersArm();
        resetEncodersElevator();

    }
    public void resetEncodersArm(){
        leftArm.encoder.reset();
        rightArm.encoder.reset();

    }
    public double getArmEncoder(){
        return (leftArm.encoder.getPosition()+rightArm.encoder.getPosition())/2.0;
    }
    public void resetEncodersElevator(){
        leftElevator.encoder.reset();
        rightElevator.encoder.reset();
    }
    public double getElevatorEncoder(){
        return (leftElevator.encoder.getPosition()+rightElevator.encoder.getPosition())/2.0;
    }
    public void setArm(double power){
        if(power==1) {
            pidfArm.updateSetpoint(maxArmPosition,kPArm,kIArm,kDArm,kFArm);

        }
        if(power==-1) {
                pidfArm.updateSetpoint(middleArmDown,kPArm,kIArm,kDArm,kFArm);


        }

    }
    public void setArmZero(){
        pidfArm.updateSetpoint(0,kPArm,kIArm,kDArm,kFArm);

    }

    public void setElevator(double power){
        if(power == 1) {
            pidElevator.update(maxElevatorPosition,kPElevator,kIElevator,kDElevator);
        }
        if(power == -1) {
            pidElevator.update(middleElevatorDown,kPElevator,kIElevator,kDElevator);
        }
    }
    public void setElevatorZero(){
        pidElevator.update(0,kPElevator,kIElevator,kDElevator);
    }

    public void updatePosition(){
        /*double currentLeftPosition = leftElevator.getCurrentPosition();
        double currentRightPosition = rightElevator.getCurrentPosition();

        // Calcula o erro de posição
        double leftError = targetPosition - currentLeftPosition;
        double rightError = targetPosition - currentRightPosition;

        // Aplica o ganho proporcional ao erro para determinar a potência
        double leftPower = kP * leftError;
        double rightPower = kP * rightError;

        leftElevator.set(leftPower);
        rightElevator.set(rightPower);

        Log.d("Arm", String.format("Left Power: %f, Right Power: %f", leftPower, rightPower));*/
    }
    public void updateTelemetry(Telemetry telemetry){
        telemetry.addData("Left arm",leftArm.encoder.getPosition());
        telemetry.addData("Right arm",rightArm.encoder.getPosition());
        telemetry.addData("Left elevator",leftElevator.encoder.getPosition());
        telemetry.addData("Right elevator",rightElevator.encoder.getPosition());

    }
    @Override
    public void periodic() {

        double armPower = pidfArm.get(getArmEncoder());
        leftArm.set(armPower);
        rightArm.set(armPower);

        Log.d("arm",String.format("Out: %f",armPower));
        double elevatorPower = pidElevator.get(getElevatorEncoder());
        leftElevator.set(elevatorPower);
        rightElevator.set(elevatorPower);

        Log.d("Elevator",String.format("Out: %f",armPower));
        // This method will be called once per scheduler run
    }
}
