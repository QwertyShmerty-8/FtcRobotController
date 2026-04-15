package org.firstinspires.ftc.teamcode.subsystems;



import static org.firstinspires.ftc.teamcode.teleop.intakeTest.offsetValueEncoder;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
    double turretAngleAdjust;

    DcMotorEx flyWheel;
    Servo hood;
    DcMotor turretMotor;
    Fullfieldshootingvalues values;
    Follower follower;

    AnalogInput turretEncoder;

    boolean isBlue;
    boolean turretOn;
    boolean hoodOn;
    double startTurretPosition;

    public static double Kpturret=0.1;
    public static double Kiturret;
    public static double Kdturret;
    public static double Kfturret=0.08;


    public static double Kp= 0.1;
    public static double KvBearing= 0.00025;
    public static double KvTurn = 0.005;
    public static double flywheeltarget;
    public static double kpflywheel;
    double lastErrorTurret;
    double lastErrorFlywheel;


    private double lastError = 0;
    private double integralSum = 0;
    private final double dt = 0.02;

    private double turretOffset =-22;

    private final double TARGET_X_BLUE=0;
    private final double TARGET_Y_BLUE=144;

    private final double TARGET_X_RED=144;


    public Turret(HardwareMap hardwareMap,  String goalColor, int x, boolean turretOnx){
        turretMotor = hardwareMap.get(DcMotor.class, "turret");
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        hood = hardwareMap.get(Servo.class, "top");

        turretEncoder = hardwareMap.get(AnalogInput.class, "encoder");


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

        double power = (target / MAX_VELOCITY) + kP * error;
        flyWheel.setPower(power);

    }

    public void setFLyWheelSpeedPID(double speed) {
        double MAX_VELOCITY=1950;
        double kP = .005;
        double target = speed;
        double error = target- flyWheel.getVelocity();

        // PID


            double error = turretDeviationNeeded - turretDeviation;

    public double turretFieldPosition(){
        double robotHeading = Math.toDegrees(follower.getHeading());
        double position = robotHeading - getTurretPosition();

        return position;

    }
    public void angularVelocityPIDF(double targetAngle){
        double dx,dy;
        if(isBlue) {
           dx = TARGET_X_BLUE -follower.getPose().getX() ;
           dy = TARGET_Y_BLUE - follower.getPose().getY();
        } else{
            dx = TARGET_X_RED - follower.getPose().getX();
            dy = TARGET_Y_RED - follower.getPose().getY();
        }
        double robotHeading = Math.toDegrees(follower.getHeading());

        // convert field target into turret frame
        double turretTarget = robotHeading-targetAngle;

        // wrap to shortest path
        turretTarget = angleWrap(turretTarget);

        // clamp to physical limits
        turretTarget = Math.max(-180, Math.min(180, turretTarget));



        double error = turretTarget -getTurretPosition();

        double steering = Kp * error;

        double robotAngularVel = Math.toDegrees(follower.getAngularVelocity());
        double angularFF = KvTurn * robotAngularVel;

        double vx = follower.getVelocity().getXComponent();
        double vy = follower.getVelocity().getYComponent();

        double distance = Math.hypot(dx, dy);
        double bearingRate;
        if (isBlue) {
         bearingRate = (vy * dx - vx * dy) / (distance * distance);
        }else{
        bearingRate = (vx * dy - vy * dx) / (distance * distance);
        }
        double bearingFF = KvBearing * Math.toDegrees(bearingRate);

        double power = steering + angularFF + bearingFF;

        power = Math.max(-1, Math.min(1, power));
    if (turretOn) {
    turretMotor.setPower(power);
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

    public void turretPIDF(double targetAngle, Follower follower) {
        double robotHeading = Math.toDegrees(follower.getHeading());
        double turretNeeded = robotHeading-targetAngle;



        turretNeeded = -Math.max(-180, Math.min(turretNeeded, 180));




        double error = turretNeeded -getTurretPosition();

        integralSum += error * dt;
        integralSum = Math.max(-50, Math.min(50, integralSum));

        double derivative = (error - lastErrorTurret) / dt;
        double ff = Kfturret * Math.signum(error);
        double output = Kpturret * error + Kiturret * integralSum + Kdturret * derivative+ff;


// Cap speed
        output = Math.max(-1, Math.min(1, output));
        turretMotor.setPower(output);

        lastErrorTurret = error;




    }
    public double error(double x, double y) {
        double targetAngle = getTargetAngle(x,y);
        double robotHeading = Math.toDegrees(follower.getHeading());
        //double turretNeeded = (targetAngle - robotHeading + 540) % 360 - 180;
       double turretNeeded = robotHeading-targetAngle ;

        double aimAngleBlue = Math.toDegrees((Math.PI / 2) + Math.atan2(x , (144 - y)));
        double turretDeviationNeeded;
        double turretDeviation = getTurretDeviationOffset();

        turretNeeded = Math.max(-180, Math.min(turretNeeded, 180));


        double error = turretNeeded - getTurretPosition();
        return error;
    }

    public double getDistance(double x, double y) {
        if (isBlue) {
            return Math.sqrt((x-TARGET_X_BLUE) * (x-TARGET_X_BLUE) + (TARGET_Y_BLUE - y) * (TARGET_Y_BLUE - y));
        } else {
            return Math.sqrt((TARGET_X_RED - x) * (TARGET_X_RED - x) + (TARGET_Y_RED - y) * (TARGET_Y_RED - y));
        }

    }

    public double getDistanceMovingWhileShooting(double x, double y, Vector velocity) {
        double vsubxt = velocity.getXComponent() * values.distancetimelut(x, y);
        double vsubyt = velocity.getYComponent() * values.distancetimelut(x, y);
        if (isBlue) {
            return Math.sqrt((x -TARGET_X_BLUE+ vsubxt) * (vsubxt -TARGET_X_BLUE+ x) + (TARGET_Y_BLUE - y - vsubyt) * (TARGET_Y_BLUE - y - vsubyt));
        } else {
            return Math.sqrt((TARGET_X_RED - x - vsubxt) * (TARGET_X_RED  - x - vsubxt) + (TARGET_Y_RED - y - vsubyt) * (TARGET_Y_RED - y - vsubyt));
        }

    }

    public double getTargetAngle(double x, double y) {
        double targetAngle;
        if (isBlue) {
            targetAngle = 90+Math.toDegrees(Math.atan2((x-TARGET_X_BLUE), (TARGET_Y_BLUE  - y)));
        } else {
            targetAngle = Math.toDegrees(Math.atan2(TARGET_Y_RED- y, TARGET_X_RED - x));
        }

        return targetAngle+turretAngleAdjust;
    }
    public void updateTurretAngleAdjust(double x){
        turretAngleAdjust = x;
    }

    public void resetTurret(){
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Reset the motor encoder
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public double getTargetAngleMovingWhileShooting(double x, double y, Vector velocity) {
        double targetAngle;
        double vsubxt = velocity.getXComponent() * values.distancetimelut(x, y);
        double vsubyt = velocity.getYComponent() * values.distancetimelut(x, y);
        if (isBlue) {
            targetAngle = (Math.PI/2)+ Math.atan2((TARGET_Y_BLUE - y - vsubyt), (x-TARGET_X_BLUE + vsubxt));
        } else {
            targetAngle = Math.atan2(TARGET_Y_RED - y - vsubyt, TARGET_X_RED - x - vsubxt);
        }
        return Math.toDegrees(targetAngle);

    }

    //Turret Getters/Setters

    public void disableFlywheeladjust(){flywheelOn =false;}
    public void disableTurretAim(){turretOn = false;}
    public void disableHoodAdjust(){hoodOn = false;}
    public void enableFlywheeladjust(){flywheelOn =true;}
    public void enableTurretAim(){turretOn = true;}
    public void enableHoodAdjust(){hoodOn = true;}

    public void setFlywheelVelocity(double velocity){
        flyWheel.setVelocity(velocity);
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
    public void setHoodAngle(double position){
        hood.setPosition(position);
    }

    public double getFlywheelVelocity(){
        return flyWheel.getVelocity();
    }
    public double getFlywheelTarget(){
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double distance = getDistance(x,y);
        return values.flywheelspeedlut(distance);
    }

    public double getTurretPosition(){
        return (turretMotor.getCurrentPosition() * 360) / (1400);


    }

    public double calculateTurretOffset(){
        double offset = + 360;
        return ((turretEncoder.getVoltage() / 3.2 * 120 + offset) % 120)-60;

    }
    public Pose getCurrentPose(){
        return follower.getPose();
    }


    //Toggle Switches
    public void switchTurretState(){turretOn = !turretOn;}
    public boolean getTurretOn(){return turretOn;}
    public void switchHoodAdjust(){hoodOn = !hoodOn;}
    public boolean getHoodAdjustOn(){return hoodOn;}





    //What needs to be called in Final Code
    public void updateFollower(Follower follower){this.follower = follower;}
    public void runTurret(){
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        Vector v = follower.getVelocity();
        double targetAngle;
        double targetDistance;
        if (movingWhileShooting){
            targetAngle = getTargetAngleMovingWhileShooting(x,y,v);
            targetDistance = getDistanceMovingWhileShooting(x,y,v);
        } else {
            targetAngle = getTargetAngle(x,y);
            targetDistance = getDistance(x,y);
        }
        if (hoodOn){
            hood.setPosition(values.hoodanglelut(targetDistance));
        }
        if (flywheelOn){
            flyWheelPidf(targetDistance);
        }
        if (turretOn){
            angularVelocityPIDF(targetAngle);
        }



    public double getTurretDeviationFromEncoder(){
        return (turretMotor.getCurrentPosition() * 360) / 1400;
    }
    public double getTurretDeviationOffset(){
        return ((turretMotor.getCurrentPosition() * 360) / 1400)-startTurretPosition;
    }
    public void updateFlywheelCoefficents(){
        flyWheelCoefficients = new PIDFCoefficients (Pflywheel,Iflywheel,Dflywheel,Fflywheel);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flyWheelCoefficients);

    public double angleWrap(double angle){
        while(angle > 180) angle -= 360;
        while(angle < -180) angle += 360;
        return angle;
    }


}

