package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    CRServo intake1;
    CRServo intake2;

    public Intake(HardwareMap hardwareMap) {
        intake1 = hardwareMap.crservo.get("intake1");
        intake2 = hardwareMap.crservo.get("intake2");
    }

    public void puxar() {
        intake1.setPower(1);
        intake2.setPower(-1);
    }

    public void devolver() {
        intake1.setPower(-1);
        intake2.setPower(1);
    }

    public void desligar() {
        intake1.setPower(0);
        intake2.setPower(0);
    }

}
