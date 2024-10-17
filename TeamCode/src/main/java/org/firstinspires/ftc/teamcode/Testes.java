package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;


@TeleOp(name="Testes")

public class Testes extends LinearOpMode
{

    @Override public void runOpMode()
    {
        // Wait for driver to press start
        while(opModeInInit()) {
            telemetry.addData(">", "Touch Play to drive");

            // Read and display sensor data
            telemetry.update();
        };
        Intake intake = new Intake(hardwareMap);

        while (opModeIsActive())
        {

            if(gamepad1.right_bumper){
                intake.puxar();
            }else if(gamepad1.left_bumper){
                intake.devolver();
            }else {
                intake.desligar();
            }

        }
        }
    }
