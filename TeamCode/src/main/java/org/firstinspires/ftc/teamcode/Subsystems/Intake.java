package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

public class Intake extends SubsystemBase {
    //private final ColorRangeSensor colorSensor;
    private final CRServo intakeRight;
    private final CRServo intakeLeft;
    private final Servo jointLeft;
    private final Servo jointRight;
    private final double power = 1;
    private final double initialPosition = 0;  // Posição inicial a 0 graus
    private final double servoDeliveryPosition = 100;       // Posição para entregar a 180 graus
    private final double servoPickupPosition = 75;           // Posição para pegar a 90 graus
    private final double SCALE_FACTOR = 255;
    private final float[] hsvValues = {0F, 0F, 0F};
    // values is a reference to the hsvValues array.
    private String alliance;
    private final ElapsedTime timer;
    private final int timePushingWrongColor = 1000;
    private final double distanceToElement = 25;
    public Intake(HardwareMap hardwareMap) {
        intakeRight = hardwareMap.crservo.get("intake1");
        intakeLeft = hardwareMap.crservo.get("intake2");
        jointLeft = hardwareMap.servo.get("bracoLeft");
        jointRight = hardwareMap.servo.get("bracoRight");

        //colorSensor = hardwareMap.get(ColorRangeSensor.class,"colorsensor");
        timer = new ElapsedTime();
        timer.reset();
    }
    public void setAlliance(String alliance){
       if(alliance.equals("Azul")){
            this.alliance = "blue";
        }
        else if(alliance.equals("Vermelho")){
            this.alliance = "red";
        }
        else{
            this.alliance = "Unknown";
        }
    }


    /*
    public String detectColor() {
        Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
    public String detectColor() {
        Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                (int) (colorSensor.green() * SCALE_FACTOR),
                (int) (colorSensor.blue() * SCALE_FACTOR),
                hsvValues);
        float hue = hsvValues[0];

        if (hue >= 180 && hue < 250) {
            return "blue";
        } else if (hue >= 50 && hue < 100) {
            return "yellow";
        } else if (hue >= 0 && hue <40) {
            return "red";
        } else {
            return "Unknown";
        }
    }

     */


    // Método para converter graus em valores entre 0 e 1 para o servo
    public double degreesToServoPosition(double degress) {
        return degress / 180.0;
    }


    public void setInitialPosition() {
        // Configura o servo na posição inicial (0 graus)
        jointLeft.setPosition(degreesToServoPosition(initialPosition));
        jointRight.setPosition(1.0 - degreesToServoPosition(initialPosition));
    }


    public void down() {
        // Configura o braço para a posição de pegar (90 graus)
        double catchPosition = degreesToServoPosition(servoPickupPosition);
        jointLeft.setPosition(catchPosition);
        jointRight.setPosition(1.0 - catchPosition);
    }

    public void up() {
        // Configura o braço para a posição de entregar (180 graus)
        double deliveryPosition = degreesToServoPosition(servoDeliveryPosition);
        jointLeft.setPosition(deliveryPosition);
        jointRight.setPosition(1.0 - deliveryPosition);
    }

    public void pull() {
        intakeRight.setPower(power);
        intakeLeft.setPower(-power);
    }

    public void push() {
        intakeRight.setPower(-power);
        intakeLeft.setPower(power);
    }

    public void powerOff() {
        if(timer.time(TimeUnit.MILLISECONDS)>timePushingWrongColor) {
            intakeRight.setPower(0);
            intakeLeft.setPower(0);
        }
    }

    public void updateTelemetry(Telemetry telemetry) {
        //String colorDetected = detectColor();
        //telemetry.addData("Cor detectada", colorDetected);
    }
    public boolean elementPresent(){
        //return colorSensor.getDistance(DistanceUnit.MM)<distanceToElement;
        return true;
    }
    @Override
    public void periodic() {

        /*
            if (((detectColor().equals("blue") && aliance.equals("red")) ||
                    (detectColor().equals("red") && aliance.equals("blue")))
                    && colorSensor.getDistance(DistanceUnit.MM) < 25) {

                if (((detectColor().equals("blue") && alliance.equals("red")) ||
                        (detectColor().equals("red") && alliance.equals("blue")))
                        && colorSensor.getDistance(DistanceUnit.MM) < distanceToElement) {
                    timer.reset();
                    push();

                }
            }
        }
         */
    }
}
