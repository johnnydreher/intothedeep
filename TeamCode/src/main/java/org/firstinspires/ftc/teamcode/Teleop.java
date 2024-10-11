/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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
    Arm arm = new Arm();
    @Override public void runOpMode()
    {
        // Initialize the drive hardware & Turn on telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrain.init(hardwareMap,false);
        arm.init(hardwareMap);
        // Wait for driver to press start
        while(opModeInInit()) {
            telemetry.addData(">", "Touch Play to drive");

            // Read and display sensor data
            telemetry.update();
        };
        Drive drive = new Drive(drivetrain);
        GamepadEx myGamepad = new GamepadEx(gamepad1);

        while (opModeIsActive())
        {
            arm.setPower((myGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)-myGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
           drivetrain.drive(myGamepad.getLeftX(),myGamepad.getLeftY(),myGamepad.getRightX(),!myGamepad.getButton(GamepadKeys.Button.B));
        }
    }
}