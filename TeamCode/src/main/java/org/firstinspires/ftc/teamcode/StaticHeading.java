package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Commands.DriveTo;
import org.firstinspires.ftc.teamcode.Subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@Config
@Autonomous(name = "Static Heading")
public class StaticHeading extends LinearOpMode {

    public static double distance = 50;
    Drivetrain drivetrain = new Drivetrain();
    Arm arm = new Arm();
    //AprilTag aprilTag = new AprilTag();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrain.init(hardwareMap,true);
        arm.init(hardwareMap);
        //aprilTag.init(hardwareMap);



        waitForStart();

       /* DriveTo drive = new DriveTo(distance,drivetrain);
        while (!drive.update()){
            updateTelemetry("Driving");
        }*/
        drivetrain.power(0);
        while(opModeIsActive()){
            updateTelemetry("End");


        }
        // Save more CPU resources when camera is no longer needed.
        //aprilTag.close();
    }
    private void updateTelemetry(String function){
        //aprilTag.telemetryAprilTag(telemetry);
        telemetry.addData("Function",function);
        telemetry.addData("Set",distance);
        telemetry.addData("Horizontal",drivetrain.getHorizontal());
        telemetry.addData("Vertical",drivetrain.getVertical());
        telemetry.addData("Gyro",drivetrain.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }




}
