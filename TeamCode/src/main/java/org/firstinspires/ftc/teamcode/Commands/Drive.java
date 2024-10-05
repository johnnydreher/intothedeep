package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

public class Drive {
    private Drivetrain dt;
    public Drive(Drivetrain dt){
        this.dt = dt;
    }
    public void drive(double y, double x, double rx, boolean fieldRelative){

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if(fieldRelative) {
            double botHeading = dt.getHeading(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double lf = (rotY + rotX + rx) / denominator;
            double lb = (rotY - rotX + rx) / denominator;
            double rf = (rotY - rotX - rx) / denominator;
            double rb = (rotY + rotX - rx) / denominator;

            dt.power(lf, lb, rf, rb);
        }
        else{
            dt.moveRobot(y,x,rx);
        }


    }
}
