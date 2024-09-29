package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Drivetrain {
    public DcMotor topLeftDriveMotor;
    public DcMotor bottomLeftDriveMotor;
    public DcMotor topRightDriveMotor;
    public DcMotor bottomRightDriveMotor;
    public DcMotor encoderVertical;
    public DcMotor encoderHorizontal;
    HardwareMap hwMap;
    static double wheelDiameter = 3.5;
    static int pulsePerRevolution = 8192;
    static double wheelPerimeter = wheelDiameter * Math.PI;
    static double distancePerTick = wheelPerimeter/pulsePerRevolution;

    private BNO055IMU imu;

    public void init(HardwareMap ahwMap) {

        /**
         * Assigns the parent hardware map to local ArtemisHardwareMap class variable
         * **/
        hwMap = ahwMap;

        /**
         * Hardware initialized and String Names are in the Configuration File for Hardware Map
         * **/

        // Control HUb
        topLeftDriveMotor = hwMap.get(DcMotor.class, "LF");
        bottomLeftDriveMotor = hwMap.get(DcMotor.class, "LB");
        topRightDriveMotor = hwMap.get(DcMotor.class, "RF");
        bottomRightDriveMotor = hwMap.get(DcMotor.class, "RB");
        encoderHorizontal = hwMap.get(DcMotor.class, "EncoderHorizontal");
        encoderVertical = hwMap.get(DcMotor.class, "EncoderVertical");

        imu = ahwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        /**
         * Allow the 4 wheel motors to be run without encoders since we are doing a time based autonomous
         * **/
        topLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        resetEncoders();

        /**
         *Since we are putting the motors on different sides we need to reverse direction so that one wheel doesn't pull us backwards
         * **/

        //THIS IS THE CORRECT ORIENTATION
        topLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        bottomLeftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        topRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        /**
         * Reverses shooter motor to shoot the correct way and same with the conveyor motor
         * **/

        /**
         * We are setting the motor 0 mode power to be brake as it actively stops the robot and doesn't rely on the surface to slow down once the robot power is set to 0
         * **/
        topLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        /**
         *The 4 mecanum wheel motors, intake, conveyor, and shooter motor/servo are set to 0 power to keep it from moving when the user presses the INIT button
         * **/
        topLeftDriveMotor.setPower(0);
        bottomLeftDriveMotor.setPower(0);
        topRightDriveMotor.setPower(0);
        bottomRightDriveMotor.setPower(0);

    }

    public void power(double output){
        topLeftDriveMotor.setPower(output);
        bottomLeftDriveMotor.setPower(output);
        topRightDriveMotor.setPower(output);
        bottomRightDriveMotor.setPower(output);
    }
    public void moveRobot(double leftStickY, double leftStickX, double rightStickX){
        /**
         * Wheel powers calculated using gamepad 1's inputs leftStickY, leftStickX, and rightStickX
         * **/
        double topLeftPower = leftStickY + leftStickX + rightStickX;
        double bottomLeftPower = leftStickY - leftStickX + rightStickX;
        double topRightPower = leftStickY - leftStickX - rightStickX;
        double bottomRightPower = leftStickY + leftStickX - rightStickX;

        /**
         * Sets the wheel's power
         * **/
        topLeftDriveMotor.setPower(topLeftPower);
        topRightDriveMotor.setPower(topRightPower);
        bottomLeftDriveMotor.setPower(bottomLeftPower);
        bottomRightDriveMotor.setPower(bottomRightPower);
    }
    public double getVertical(){
        return encoderVertical.getCurrentPosition()*distancePerTick;
    }
    public double getHorizontal(){
        return encoderHorizontal.getCurrentPosition()*distancePerTick;
    }
    public double getHeading(){
        return imu.getAngularOrientation().firstAngle;
    }
    public void resetEncoders(){
        encoderVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
}