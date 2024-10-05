package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class Drivetrain {
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public DcMotor encoderVertical;
    public DcMotor encoderHorizontal;
    HardwareMap hwMap;
    static double wheelDiameter = 3.5;
    static int pulsePerRevolution = 8192;
    static double wheelPerimeter = wheelDiameter * Math.PI;
    static double distancePerTick = wheelPerimeter/pulsePerRevolution;

    private IMU imu         = null;      // Control/Expansion Hub IMU

    public void init(HardwareMap ahwMap, boolean usingEncoderMotor) {

        /**
         * Assigns the parent hardware map to local ArtemisHardwareMap class variable
         * **/
        hwMap = ahwMap;

        /**
         * Hardware initialized and String Names are in the Configuration File for Hardware Map
         * **/

        // Control HUb
        leftFrontDrive = hwMap.get(DcMotor.class, "LF");
        leftBackDrive = hwMap.get(DcMotor.class, "LB");
        rightFrontDrive = hwMap.get(DcMotor.class, "RF");
        rightBackDrive = hwMap.get(DcMotor.class, "RB");
        encoderHorizontal = hwMap.get(DcMotor.class, "EH");
        encoderVertical = hwMap.get(DcMotor.class, "EV");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        /**
         * Allow the 4 wheel motors to be run without encoders since we are doing a time based autonomous
         * **/
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(usingEncoderMotor) {
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else{
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        resetEncoders();

        /**
         *Since we are putting the motors on different sides we need to reverse direction so that one wheel doesn't pull us backwards
         * **/

        //THIS IS THE CORRECT ORIENTATION
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        /**
         * Reverses shooter motor to shoot the correct way and same with the conveyor motor
         * **/

        /**
         * We are setting the motor 0 mode power to be brake as it actively stops the robot and doesn't rely on the surface to slow down once the robot power is set to 0
         * **/
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        /**
         *The 4 mecanum wheel motors, intake, conveyor, and shooter motor/servo are set to 0 power to keep it from moving when the user presses the INIT button
         * **/
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

    }

    public void power(double output){
        leftFrontDrive.setPower(output);
        leftBackDrive.setPower(output);
        rightFrontDrive.setPower(output);
        rightBackDrive.setPower(output);
    }
    public void power(double lf, double lb, double rf, double rb){
        leftFrontDrive.setPower(lf);
        leftBackDrive.setPower(lb);
        rightFrontDrive.setPower(rf);
        rightBackDrive.setPower(rb);
    }
    public void moveRobot(double drive, double strafe, double yaw){
        /**
         * Wheel powers calculated using gamepad 1's inputs leftStickY, leftStickX, and rightStickX
         * **/
        double lF = drive - strafe - yaw;
        double rF = drive + strafe + yaw;
        double lB = drive + strafe - yaw;
        double rB = drive - strafe + yaw;
        double max = Math.max(Math.abs(lF), Math.abs(rF));
        max = Math.max(max, Math.abs(lB));
        max = Math.max(max, Math.abs(rB));

        //normalize the motor values
        if (max > 1.0) {
            lF /= max;
            rF /= max;
            lB /= max;
            rB /= max;
        }
        /**
         * Sets the wheel's power
         * **/
        leftFrontDrive.setPower(lF);
        rightFrontDrive.setPower(rF);
        leftBackDrive.setPower(lB);
        rightBackDrive.setPower(rB);
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
}