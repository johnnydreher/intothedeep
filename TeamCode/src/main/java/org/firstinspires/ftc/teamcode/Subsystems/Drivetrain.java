package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import com.arcrobotics.ftclib.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    private Motor leftFrontDrive;
    private Motor leftBackDrive;
    private Motor rightFrontDrive;
    private Motor rightBackDrive;
    private DcMotor encoderVertical;
    private DcMotor encoderHorizontal;
    private HardwareMap hwMap;
    private static final double wheelDiameter = 3.5;
    private static final int pulsePerRevolution = 8192;
    private static final double wheelPerimeter = wheelDiameter * Math.PI;
    private static final double distancePerTick = wheelPerimeter/pulsePerRevolution;
    private static VoltageSensor voltage;
    private IMU imu         = null;      // Control/Expansion Hub IMU
    private MecanumDrive mecanum =  null;
    public void init(HardwareMap ahwMap, boolean usingEncoderMotor) {

        /**
         * Assigns the parent hardware map to local ArtemisHardwareMap class variable
         * **/
        hwMap = ahwMap;

        /**
         * Hardware initialized and String Names are in the Configuration File for Hardware Map
         * **/
        voltage = hwMap.voltageSensor.iterator().next();
        // Control HUb
        leftFrontDrive = new Motor(hwMap, "LF");
        leftBackDrive = new Motor(hwMap, "LB");
        rightFrontDrive = new Motor(hwMap, "RF");
        rightBackDrive = new Motor(hwMap, "RB");
        encoderHorizontal = hwMap.get(DcMotor.class, "LF");
        encoderVertical = hwMap.get(DcMotor.class, "LB");

        mecanum = new MecanumDrive(leftFrontDrive,rightFrontDrive,leftBackDrive,rightBackDrive);
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        mecanum.stop();
        resetEncoders();



    }

    public void power(double output){
        mecanum.driveWithMotorPowers(output,output,output,output);
    }
    public void power(double lf, double lb, double rf, double rb){
        mecanum.driveWithMotorPowers(lf,rf,lb,rb);
    }
    public void moveRobot(double drive, double strafe, double yaw){
       mecanum.driveRobotCentric(strafe,drive,yaw);
    }
    public void driveFieldCentric(double drive, double strafe, double yaw){
        mecanum.driveFieldCentric(strafe,drive,yaw, getHeading(AngleUnit.DEGREES));
    }

    public void drive(double drive, double strafe, double yaw, boolean fieldCentric){
        if(fieldCentric){
            driveFieldCentric(drive,strafe,yaw);
        }
       else{
            moveRobot(drive,strafe,yaw);
        }
    }
    public double getVertical(){
        return encoderVertical.getCurrentPosition()*distancePerTick;
    }
    public double getHorizontal(){
        return encoderHorizontal.getCurrentPosition()*distancePerTick;
    }
    public double getHeading(AngleUnit unit){
        return imu.getRobotYawPitchRollAngles().getYaw(unit);
    }
    public void resetEncoders(){
        encoderVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu.resetYaw();


    }
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
}
}