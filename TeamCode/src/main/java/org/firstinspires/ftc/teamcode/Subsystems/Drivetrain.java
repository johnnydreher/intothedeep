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

    private Motor.Encoder encoderVertical;
    private Motor.Encoder encoderHorizontal;
    private static final double wheelDiameter = 3.5;
    private static final int pulsePerRevolution = 8192;
    private static final double wheelPerimeter = wheelDiameter * Math.PI;
    private static final double distancePerTick = wheelPerimeter/pulsePerRevolution;
    private IMU imu         = null;      // Control/Expansion Hub IMU
    private MecanumDrive mecanum =  null;
    public void init(HardwareMap ahwMap, boolean usingEncoderMotor) {

        /**
         * Assigns the parent hardware map to local ArtemisHardwareMap class variable
         * **/

        /**
         * Hardware initialized and String Names are in the Configuration File for Hardware Map
         * **/
        VoltageSensor voltage = ahwMap.voltageSensor.iterator().next();
        // Control HUb
        Motor leftFrontDrive = new Motor(ahwMap, "LF");
        Motor leftBackDrive = new Motor(ahwMap, "LB");
        Motor rightFrontDrive = new Motor(ahwMap, "RF");
        Motor rightBackDrive = new Motor(ahwMap, "RB");
        encoderHorizontal = rightBackDrive.encoder;
        encoderVertical = leftFrontDrive.encoder;

        encoderVertical.setDirection(Motor.Direction.REVERSE);

        encoderVertical.setDistancePerPulse(distancePerTick);
        encoderHorizontal.setDistancePerPulse(distancePerTick);
        mecanum = new MecanumDrive(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = ahwMap.get(IMU.class, "imu");
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
        return encoderVertical.getDistance();
    }
    public double getHorizontal(){
        return encoderHorizontal.getDistance();
    }
    public double getHeading(AngleUnit unit){
        return imu.getRobotYawPitchRollAngles().getYaw(unit);
    }
    public void resetEncoders(){
        encoderHorizontal.reset();
        encoderVertical.reset();
        imu.resetYaw();
    }
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
}
}