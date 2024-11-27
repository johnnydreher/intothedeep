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
import org.firstinspires.ftc.teamcode.Commands.DriveToTag;
import org.firstinspires.ftc.teamcode.Commands.StrafeTo;
import org.firstinspires.ftc.teamcode.Commands.TurnTo;
import org.firstinspires.ftc.teamcode.Constants.DistanceConstant;
import org.firstinspires.ftc.teamcode.Subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@Config
@Autonomous(name = "Static Heading")
public class StaticHeading extends LinearOpMode {

    public static double distance = 50;
    Drivetrain drivetrain = new Drivetrain();
    Arm arm = new Arm(true);
    AprilTag aprilTag = new AprilTag();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrain.init(hardwareMap,true);
        arm.init(hardwareMap);
        aprilTag.init(hardwareMap);



        while (!isStarted()) {
            aprilTag.telemetryAprilTag(telemetry);
            telemetry.update();
        }
        DriveToTag dtt = new DriveToTag(drivetrain,aprilTag,1);
        dtt.initialize();
        while (!dtt.isFinished()){
            dtt.execute();
            aprilTag.telemetryAprilTag(telemetry);
            updateTelemetry("DriveToTag");

        }
        drivetrain.power(0);
        while(opModeIsActive()){
            aprilTag.telemetryAprilTag(telemetry);
            updateTelemetry("End");


        }
        // Save more CPU resources when camera is no longer needed.
        aprilTag.close();
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
