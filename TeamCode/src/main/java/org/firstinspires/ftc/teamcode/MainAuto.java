/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;

import com.qualcomm.ftcrobotcontroller.BuildConfig;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Commands.Autos;
import org.firstinspires.ftc.teamcode.Commands.Drive;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Blinkin;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

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
    Arm arm = new Arm(true);
    Intake intake;
    Blinkin blinkin;
    List<Command> auto4Pieces;
    int actualCommand = 0;
    Command cmd;
    @Override public void initialize()
    {
        // Initialize the drive hardware & Turn on telemetry

        arm.init(hardwareMap);
        intake = new Intake(hardwareMap);
        blinkin = new Blinkin(hardwareMap);
        String alliance = "Vermelho";
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrain.init(hardwareMap,false);
        intake.setAlliance(alliance);
        auto4Pieces = Autos.Auto4Pieces(drivetrain, arm, intake);
        while(opModeInInit()) {
            alliance = intake.detectColor();
            blinkin.setAlianca(alliance);
            intake.updateTelemetry(telemetry);
            telemetry.addData("data", BuildConfig.APP_BUILD_TIME);
            telemetry.addData("Autonomo principal", " 4 peças");
            telemetry.addData("Aliança Atual", alliance);
            telemetry.update();
            intake.setAlliance(alliance);
        }



    }
    @Override
    public void run(){
        if(cmd == null){
            cmd = auto4Pieces.get(0);
            cmd.initialize();
        }
        cmd.execute();
        if(cmd.isFinished()){
            cmd.end(false);
            actualCommand++;
            cmd = auto4Pieces.get(actualCommand);
            cmd.initialize();
        }
        //arm.updateTelemetry(telemetry);
        //intake.updateTelemetry(telemetry);
        drivetrain.periodic();
        arm.periodic();
        intake.periodic();
        telemetry.addData("Command",cmd.getName());
        telemetry.addData("X",drivetrain.getVertical());
        telemetry.addData("Y",drivetrain.getHorizontal());
        telemetry.addData("Angle",drivetrain.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }
}