package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Objects;

public class Blinkin extends SubsystemBase {
    private Servo pwmServo;

    private double apagado = 0;
    private double vermelho = 0.21;
    private double azul = 0.35;
    private double laranja = 0.47;
    private double piscaAmarelo = 0.63;
    private double piscaAzul = 0.77;
    private double piscaVermelho = 0.91;
    private double rainbow = 1;

    public Blinkin(HardwareMap hardwareMap){
        pwmServo = hardwareMap.servo.get("blinkin");
    }
    public void setAzul(){
        pwmServo.setPosition(azul);
    }
    public void setVermelho(){
        pwmServo.setPosition(vermelho);
    }
    public void setApagado(){
        pwmServo.setPosition(apagado);
    }
    public void setLaranja(){
        pwmServo.setPosition(laranja);
    }
    public void setPiscaAmarelo(){
        pwmServo.setPosition(piscaAmarelo);
    }
    public void setPiscaAzul(){
        pwmServo.setPosition(piscaAzul);
    }
    public void setRainbow(){
        pwmServo.setPosition(rainbow);
    }
    public void setPiscaVermelho(){
        pwmServo.setPosition(piscaVermelho);
    }
    public void setAlianca(String alianca){
        if(Objects.equals(alianca, "blue")){
            setAzul();
        }
        if(Objects.equals(alianca, "ref")){
            setVermelho();
        }

    }
}
