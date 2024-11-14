package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;


@TeleOp(name="Testes")

public class Testes extends LinearOpMode
{



    @Override public void runOpMode() {
        Intake intake = new Intake(hardwareMap);

        while (opModeInInit()) {
            telemetry.addData(">", "Touch Play to drive");
            telemetry.update();

            // Read and display sensor data

        }

        while (opModeIsActive()) {

            intake.updateTelemetry(telemetry);
            telemetry.update();

        }
    }
    }
