package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Blinkin;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;


@TeleOp(name="Testes")

public class Testes extends LinearOpMode
{



    @Override public void runOpMode() {
        Blinkin blinkin  = new Blinkin(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        while (opModeInInit()) {
            telemetry.addData(">", "Touch Play to drive");
            telemetry.update();

            // Read and display sensor data

        }

        while (opModeIsActive()) {
            String color = intake.detectColor();
            if(color=="blue"){
                blinkin.setAzul();
                gamepad1.setLedColor(0,0,1.0,60000);
            }
            else if (color=="red"){
                blinkin.setVermelho();
                gamepad1.setLedColor(1.0,0,0,60000);
            }
            else if(color=="yellow"){
                blinkin.setPiscaAmarelo();
                gamepad1.setLedColor(1.0,1.0,0,60000);
            }
            else{
                blinkin.setApagado();
                gamepad1.setLedColor(0,1.0,0,60000);
            }
            intake.updateTelemetry(telemetry);
            telemetry.update();

        }
    }
    }
