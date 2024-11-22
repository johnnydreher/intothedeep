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
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

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
        Intake intake = new Intake(hardwareMap);

        String alianca = "Indefinida";

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrain.init(hardwareMap,false);
        // Wait for driver to press start
        while(opModeInInit()) {
            telemetry.addData(">", "Touch Play to drive");
            telemetry.addData("Seleção de Aliança", "Pressione A para Azul, B para Vermelha");
            telemetry.addData("Aliança Atual", alianca);
            telemetry.update();

            // Verifica os botões A e B do gamepad para alternar entre azul e vermelha
            if (gamepad2.a) {
                alianca = "Azul";
            } else if (gamepad2.b) {
                alianca = "Vermelha";
            }

            intake.setAlliance(alianca);

            // Read and display sensor data
            telemetry.update();
            intake.setInitialPosition();
        }
        Drive drive = new Drive(drivetrain);

        while (opModeIsActive())
        {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            boolean fieldRelative = !gamepad1.a;
           drive.drive(y,x,rx,fieldRelative);

           if(gamepad1.start){
               drivetrain.resetEncoders();
           }

           if(gamepad2.dpad_up){
                arm.setElevatorZero();
            }
           if(gamepad2.dpad_down){
            arm.setArmZero();
        }

           if(gamepad2.left_bumper){
                arm.setArm(1);
           } else if (gamepad2.right_bumper) {
               arm.setArm(-1);
           }

           if(gamepad2.right_trigger > 0.5){
               arm.setElevator(1);
           }else if (gamepad2.left_trigger > 0.1) {
               arm.setElevator(-1);
           }

           /*if(gamepad2.left_bumper){
                arm.setPower(0.75);
           } else if (gamepad2.right_bumper) {
               arm.setPower(-0.75);
           }else{arm.setPower(0);}*/

           arm.periodic();
           arm.updateTelemetry(telemetry);
           intake.updateTelemetry(telemetry);
           telemetry.update();



            if(gamepad2.triangle){
                intake.pull();
            }else if(gamepad2.circle){
                intake.push();
            }else {
                intake.powerOff();
            }

            if(gamepad2.square){
                intake.up();
            }
            if(gamepad2.cross){
                intake.down();
            }

        }
    }
}