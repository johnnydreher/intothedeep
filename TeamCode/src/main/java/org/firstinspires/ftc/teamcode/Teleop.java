/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

import java.util.Locale;

/*
 * This OpMode illustrates a teleop OpMode for an Omni robot.
 * An external "Robot" class is used to manage all motor/sensor interfaces, and to assist driving functions.
 * The IMU gyro is used to stabilize the heading when the operator is not requesting a turn.
 */

@TeleOp(name="Teleop")

public class Teleop extends LinearOpMode
{

    Drivetrain drivetrain = new Drivetrain();

    @Override public void runOpMode()
    {
        // Initialize the drive hardware & Turn on telemetry
        Arm arm = new Arm();
        arm.init(hardwareMap);
        //
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrain.init(hardwareMap,false);
        // Wait for driver to press start
        while(opModeInInit()) {
            telemetry.addData(">", "Touch Play to drive");

            // Read and display sensor data
            telemetry.update();
        }
        Drive drive = new Drive(drivetrain);
        while (opModeIsActive())
        {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            boolean fieldRelative = !gamepad1.a;
           drive.drive(y,x,rx,fieldRelative);
           if(gamepad1.b){
               drivetrain.resetEncoders();
           }

           if(gamepad1.left_bumper){
                arm.setPower(1);
           } else if (gamepad1.right_bumper) {
               arm.setPower(-1);
           }else{arm.setPower(0);}

           if(gamepad1.right_trigger > 0.5){
               arm.setPowerE(1);
           }else if (gamepad1.left_trigger > 0.1){
               arm.setPowerE(-1);
           }else{
               arm.setPowerE(0);
           }
        }
    }
}