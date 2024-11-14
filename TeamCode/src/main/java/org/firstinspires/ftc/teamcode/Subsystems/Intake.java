package org.firstinspires.ftc.teamcode.Subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import android.graphics.Color;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake extends SubsystemBase {
    ColorRangeSensor sensorColor;
    CRServo intake1;
    CRServo intake2;
    Servo bracoLeft;
    Servo bracoRight;
    double power = 1;
    double initialPosition = 0;  // Posição inicial a 0 graus
    double entragar = 100;       // Posição para entregar a 180 graus
    double pegar = 75;           // Posição para pegar a 90 graus
    final double SCALE_FACTOR = 255;
    float hsvValues[] = {0F, 0F, 0F};
    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    public String Alianca;

    public Intake() {}
    public void initSensor(HardwareMap hardwareMap){
        sensorColor = hardwareMap.get(ColorRangeSensor.class, "colorsensor");
    }

    public void initServos(HardwareMap hardwareMap){
        intake1 = hardwareMap.crservo.get("intake1");
        intake2 = hardwareMap.crservo.get("intake2");
        bracoLeft = hardwareMap.servo.get("bracoLeft");
        bracoRight = hardwareMap.servo.get("bracoRight");
    }


    public String detectColor() {
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        float hue = hsvValues[0];

        if (hue >= 180 && hue < 250) {
            return "Azul";
        } else if (hue >= 50 && hue < 100) {
            return "Amarelo";
        } else if (hue >= 0 && hue <40  ) {
            return "Vermelho";
        } else {
            return "Unknown";
        }
    }


    // Método para converter graus em valores entre 0 e 1 para o servo
    public double degreesToServoPosition(double graus) {
        return graus / 180.0;
    }


    public void setInitialPosition() {
        // Configura o servo na posição inicial (0 graus)
        bracoLeft.setPosition(degreesToServoPosition(initialPosition));
        bracoRight.setPosition(1.0 - degreesToServoPosition(initialPosition));
    }


    public void desce() {
        // Configura o braço para a posição de pegar (90 graus)
        double posicaoPegar = degreesToServoPosition(pegar);
        bracoLeft.setPosition(posicaoPegar);
        bracoRight.setPosition(1.0 - posicaoPegar);
    }

    public void sobe() {
        // Configura o braço para a posição de entregar (180 graus)
        double posicaoEntragar = degreesToServoPosition(entragar);
        bracoLeft.setPosition(posicaoEntragar);
        bracoRight.setPosition(1.0 - posicaoEntragar);
    }

    public void puxar() {
        intake1.setPower(power);
        intake2.setPower(-power);
    }

    public void devolver() {
        intake1.setPower(-power);
        intake2.setPower(power);
    }

    // corrigir para aliança vermelha
    public void verificarAlianca(){
        if("Azul".equals(Alianca) && "Vermelho".equals(detectColor())){
            puxar();
        }else if("Vermelho".equals(Alianca) && "Azul".equals(detectColor())){
            puxar();
        }else {
            desligar();
        }
    }


    public void desligar() {
        intake1.setPower(0);
        intake2.setPower(0);
    }

    public void updateTelemetry(Telemetry telemetry) {
        String colorDetected = detectColor();
        telemetry.addData("Cor detectada", colorDetected);
        telemetry.addData("Distance",sensorColor.getDistance(DistanceUnit.MM));

    }
}
