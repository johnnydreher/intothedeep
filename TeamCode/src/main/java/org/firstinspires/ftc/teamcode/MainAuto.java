/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.ftcrobotcontroller.BuildConfig;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.Autos.Auto4Pieces;
import org.firstinspires.ftc.teamcode.Commands.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

/*
 * This OpMode illustrates a teleop OpMode for an Omni robot.
 * An external "Robot" class is used to manage all motor/sensor interfaces, and to assist driving functions.
 * The IMU gyro is used to stabilize the heading when the operator is not requesting a turn.
 */

@Autonomous(name="Auto Principal")

public class MainAuto extends CommandOpMode
{

    Drivetrain drivetrain = new Drivetrain();
    Drive drive;
    Arm arm = new Arm();
    Intake intake;
    @Override public void initialize()
    {
        // Initialize the drive hardware & Turn on telemetry

        arm.init(hardwareMap);
        intake = new Intake(hardwareMap);

        String aliance = "Vermelho";
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrain.init(hardwareMap,false);
        intake.setAlliance(aliance);
        Auto4Pieces auto4Pieces = new Auto4Pieces(drivetrain, arm, intake);
        while(opModeInInit()) {
            telemetry.addData("data", BuildConfig.APP_BUILD_TIME);
            telemetry.addData("Autonomo principal", " 4 peças");
            telemetry.addData("Aliança Atual", aliance);
            telemetry.update();

        }
        schedule(auto4Pieces);

    }
    @Override
    public void run(){
        CommandScheduler.getInstance().run();
        arm.updateTelemetry(telemetry);
        intake.updateTelemetry(telemetry);
        telemetry.update();
    }
}