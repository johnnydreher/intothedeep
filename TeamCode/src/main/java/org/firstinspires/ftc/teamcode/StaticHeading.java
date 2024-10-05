package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;


@TeleOp(name = "Static Heading")
public class StaticHeading extends LinearOpMode {


    Drivetrain drivetrain = new Drivetrain();

    ElapsedTime timer = new ElapsedTime();
    AprilTag aprilTag = new AprilTag();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrain.init(hardwareMap);
        aprilTag.init(hardwareMap);

        ElapsedTime stopTime   = new ElapsedTime();  // Use for timeouts.

        waitForStart();


        drivetrain.power(0);
        while(opModeIsActive()){

            updateTelemetry("End");


        }
        // Save more CPU resources when camera is no longer needed.
        aprilTag.close();
    }
    private void updateTelemetry(String function){
        aprilTag.telemetryAprilTag(telemetry);
        telemetry.addData("Function",function);

        telemetry.addData("Horizontal",drivetrain.getHorizontal());
        telemetry.addData("Vertical",drivetrain.getVertical());
        telemetry.addData("Gyro",drivetrain.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }




}
