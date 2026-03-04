package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.myConstants.Turret.KdTurret;
import static org.firstinspires.ftc.teamcode.myConstants.Turret.KiTurret;
import static org.firstinspires.ftc.teamcode.myConstants.Turret.KpTurret;
import static org.firstinspires.ftc.teamcode.myConstants.Turret.Pflywheel;
import static org.firstinspires.ftc.teamcode.myConstants.Turret.Iflywheel;
import static org.firstinspires.ftc.teamcode.myConstants.Turret.Dflywheel;
import static org.firstinspires.ftc.teamcode.myConstants.Turret.Fflywheel;
import static org.firstinspires.ftc.teamcode.myConstants.Turret.TURRET_MAX;
import static org.firstinspires.ftc.teamcode.myConstants.Turret.TURRET_MIN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Helperfunctions.Fullfieldshootingvalues;
@Config

public class Turret {

    DcMotorEx flyWheel;
    Servo hood;
    DcMotor turretMotor;
    Fullfieldshootingvalues values;

    boolean isBlue;
    boolean turretOn;
    boolean hoodOn;
    double startTurretPosition;





    private double lastError = 0;
    private double integralSum = 0;
    private final double dt = 0.02;
    PIDFCoefficients flyWheelCoefficients = new PIDFCoefficients (Pflywheel,Iflywheel,Dflywheel,Fflywheel);

    //aimAngleBlue = Math.toDegrees((Math.PI / 2) + Math.atan(pos.x / (144 - pos.y)));

    //aimAngleRed = Math.toDegrees(Math.atan((144 - pos.y) / (144 - pos.x)));
    // turretDeviationNeeded = aimAngleBlue - pos.h;

    public Turret(HardwareMap hardwareMap,  String goalColor, int x, boolean turretOnx){
        turretMotor = hardwareMap.get(DcMotor.class, "turret");
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        hood = hardwareMap.get(Servo.class, "top");


        isBlue = goalColor.equalsIgnoreCase("blue");
        turretOn = turretOnx;
        hoodOn = true;

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                // Reset the motor encoder
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flyWheelCoefficients);
        turretMotor.setDirection(DcMotor.Direction.REVERSE);
        startTurretPosition = x;

        values = new Fullfieldshootingvalues(goalColor);
    }

    public void aimTurret(double x, double y, double h){
        double aimAngleBlue = Math.toDegrees((Math.PI / 2) + Math.atan2((144 - y),(x-9.8)));
        double turretDeviationNeeded;
        double turretDeviation = getTurretDeviationOffset();

        double aimAngleRed = Math.toDegrees(Math.atan2((144 - y) ,(130 - x)));
        if (isBlue){
            turretDeviationNeeded = aimAngleBlue - h;


        } else{
            turretDeviationNeeded = aimAngleRed-h;
        }

        if ( Math.abs(turretDeviationNeeded - turretDeviation)<1||turretOn==false) {
            turretMotor.setPower(0);
            lastError = 0;
            integralSum =0;

        } else {


            double error = turretDeviationNeeded - turretDeviation;

            // shortest path

// Clamp target inside ±200
            double target = Math.max(TURRET_MIN, Math.min(TURRET_MAX, turretDeviationNeeded));

// PID
            integralSum += error * dt;
            integralSum = Math.max(-50, Math.min(50, integralSum));

            double derivative = (error - lastError) / dt;
            double output = KpTurret*error + KiTurret*integralSum + KdTurret*derivative;

// Soft limit protection

// Cap speed
            output = Math.max(-0.75, Math.min(0.75, output));

            turretMotor.setPower(output);
            lastError = error;


        }


    }

    public void autoHoodAnglelut(double x, double y){
        if (hoodOn == true) {
            hood.setPosition(values.hoodanglelut(x, y));
        }
    }

    public void disableHoodAdjust(){
        hoodOn=false;
    }
    public void enableHoodAdjust(){
        hoodOn=true;
    }
    public void switchHoodAdjust(){
        hoodOn= !hoodOn;
    }
    public boolean getHoodAdjustOn(){
        return hoodOn;
    }
    public void aimTurretGreaterthan360(double x, double y, double h){
        double aimAngleBlue = Math.toDegrees((Math.PI / 2) + Math.atan2(x , (144 - y)));
        double turretDeviationNeeded;
        double turretDeviation = getTurretDeviationOffset();

        double aimAngleRed = Math.toDegrees(Math.atan2((144 - y) ,(144 - x)));

        if (isBlue){
           turretDeviationNeeded = aimAngleBlue - h;


        } else{
            turretDeviationNeeded = aimAngleRed-h;
        }

        if ( Math.abs(turretDeviationNeeded - turretDeviation)<1||turretOn==false) {
            turretMotor.setPower(0);
            lastError = 0;
            integralSum =0;

        } else {

            double turretAngle = (turretMotor.getCurrentPosition() * 360.0) / 1400.0;

            double error = turretDeviationNeeded - turretAngle;

            if (error > 180)  error -= 360;
            if (error < -180) error += 360;
            // shortest path

// Clamp target inside ±200
            double target = Math.max(TURRET_MIN, Math.min(TURRET_MAX, turretDeviationNeeded));

// PID
            integralSum += error * dt;
            integralSum = Math.max(-50, Math.min(50, integralSum));

            double derivative = (error - lastError) / dt;
            double output = KpTurret*error + KiTurret*integralSum + KdTurret*derivative;

// Soft limit protection
            if (turretAngle >= TURRET_MAX && output > 0) output = 0;
            if (turretAngle <= TURRET_MIN && output < 0) output = 0;

// Cap speed
            output = Math.max(-0.75, Math.min(0.75, output));

            turretMotor.setPower(output);
            lastError = error;


        }


    }
    public void aimTurretOriginal(double x, double y, double h){
        double aimAngleRed = Math.toDegrees(Math.atan2((144 - y) , (144 - x)));


        double aimAngleBlue = Math.toDegrees((Math.PI / 2) + Math.atan2(x , (144 - y)));
        double turretDeviationNeeded;
        double turretDeviation = getTurretDeviationOffset();


        if (isBlue){
            turretDeviationNeeded = aimAngleBlue - h;


        } else{
            turretDeviationNeeded = aimAngleRed-h;
        }


        //turret PID

        if (Math.abs(turretDeviationNeeded - turretDeviation) < 3 || Math.abs(turretDeviationNeeded) > 150||turretOn==false) {
            turretMotor.setPower(0);
            integralSum = 0;

        } else {


            double currentAngle = turretDeviation + h;

            //  shortest-path error
            double error = turretDeviationNeeded - turretDeviation;

            integralSum += error * dt;
            double derivative = (error - lastError) / dt;

            double output = (KpTurret * error) + (KiTurret * integralSum) + (KdTurret * derivative);

            // Optional but recommended
            output = Math.max(-0.75, Math.min(0.75, output));

            turretMotor.setPower(output);

            lastError = error;

        }
    }
    public double getTargetBlue(double x,double y){
        return Math.toDegrees((Math.PI / 2) + Math.atan2((144 - y),(x-9.8)));
    }
    public double getTargetRed(double x,double y){

        return Math.toDegrees(Math.atan2((144 - y) ,(130 - x)));
    }

    public void disableTurret(){
        turretOn = false;
    }
    public void enableTurret(){
        turretOn = true;
    }

    public void fullFieldShootingHoodLinear (double x, double y, double h){
        double distance;

        if (isBlue == true){
            distance = Math.sqrt(x*x + (144-y)*(144-y));
        }else {
            distance = Math.sqrt((144-x)*(144-x) + (144-y)*(144-y));
        }


        hood.setPosition(-0.00102503*distance+0.726524);

        //insert Equation Here

    }




    public void switchTurretState(){
        turretOn=!turretOn;

    }


    public boolean getTurretOn(){
        return turretOn;
    }
    public void shootWhileMoving(){}
    public double getFlyWheelSpeed(){
        return flyWheel.getVelocity();
    }
    public void setFlyWheelSpeed(double fs){
        flyWheel.setVelocity (fs);
    }

    public void setHoodAngle(double ha){
        hood.setPosition(ha);
    }
    public double getHoodAngle(){return hood.getPosition();}


    public double getTurretDeviationFromEncoder(){
        return (turretMotor.getCurrentPosition() * 360) / 1400;
    }
    public double getTurretDeviationOffset(){
        return ((turretMotor.getCurrentPosition() * 360) / 1400)-startTurretPosition;
    }
    public void updateFlywheelCoefficents(){
        flyWheelCoefficients = new PIDFCoefficients (Pflywheel,Iflywheel,Dflywheel,Fflywheel);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flyWheelCoefficients);

    }
    public void setFlywheelHoodlut(double x, double y){
        flyWheel.setVelocity(values.flywheelspeedlut( x,y));
        hood.setPosition(values.hoodanglelut(x,y));
    }






}
