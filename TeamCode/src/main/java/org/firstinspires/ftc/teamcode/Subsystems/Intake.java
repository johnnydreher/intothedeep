package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends SubsystemBase {

    CRServo intake1;
    CRServo intake2;
    Servo bracoLeft;
    Servo bracoRight;
    double power = 1;
    double initialPosition = 0;
    double entragar = 180;
    double pegar = 90;

    public Intake(HardwareMap hardwareMap) {
        intake1 = hardwareMap.crservo.get("intake1");
        intake2 = hardwareMap.crservo.get("intake2");
        bracoLeft = hardwareMap.servo.get("bracoLeft");
        bracoRight = hardwareMap.servo.get("bracoRight");
    }

    public void puxar() {
        intake1.setPower(power);
        intake2.setPower(-power);
    }

    public void devolver() {
        intake1.setPower(-power);
        intake2.setPower(power);
    }

    public void desligar() {
        intake1.setPower(0);
        intake2.setPower(0);
    }

    public double Degrees(double graus){
        return graus / 180;
    }


    public void setInitialPosition() {

        bracoLeft.setPosition(1.0 - initialPosition);
        bracoRight.setPosition(initialPosition);
    }

    public void desce() {
        double grauDesce = Degrees(pegar);
        bracoRight.setPosition(grauDesce);
        bracoLeft.setPosition(1.0 - grauDesce);
    }

    public void sobe() {
        double grauSobe = Degrees(entragar);
        bracoRight.setPosition(1.0 - grauSobe);
        bracoLeft.setPosition(grauSobe);
    }

}
