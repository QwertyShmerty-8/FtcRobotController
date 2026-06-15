package org.firstinspires.ftc.teamcode.subsystems;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Helperfunctions.Fullfieldshootingvalues;

@Config

public class Spindex {
    DcMotor spindex;
    AnalogInput encoder;

    private double lastError = 0;
    private double integralSum = 0;
    private final double dt = 0.02;

    public static double KpSpindex = 0.0065;
    public static double KiSpindex = 0;
    public static double KdSpindex =0;
    public static double KfSpindex =0.01;

    public ElapsedTime runTimer;
    public ElapsedTime time;

    Fullfieldshootingvalues values;

    double startEncoderCounts = 0;
    boolean rotating = false;
    int encoderCountsNeeded = 0;


    public Spindex(HardwareMap hardwareMap) {
        encoder = hardwareMap.get(AnalogInput.class, "encoder");
        spindex = hardwareMap.get(DcMotorEx.class, "spindexer");
        spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        runTimer = new ElapsedTime();
        time = new ElapsedTime();



    }

    public void startRotate(int rotations){
        startEncoderCounts = spindex.getCurrentPosition();
        encoderCountsNeeded = 288 * rotations;
        rotating = true;
    }

    public void updateRotate(){

        if(!rotating) return;

        if(Math.abs(spindex.getCurrentPosition() - startEncoderCounts) < encoderCountsNeeded){
            runSpindexToggle(1);
        } else{
            time.reset();
            rotating = false;
            goToPosition(344);
        }
    }

    public boolean get1secAfterRotate(){
        return (time.seconds()>3&& rotating == false);
    }

    public void setSpindexPower(double power){
        spindex.setPower(power);

    }
    public boolean getIsRotating(){
        return rotating;
    }
    public double getSpindexMotorCounts(){
        return spindex.getCurrentPosition();
    }


    public double getPosition(){
        return encoder.getVoltage() / 3.2 * 360;
    }

    public void goToPosition(double target) {
        double raw = encoder.getVoltage() / 3.2 * 360;
        raw = ((raw % 360) + 360) % 360;

        double adjusted = raw;
        double position = (adjusted + 360) % 360;
        double errorForward = (target - position + 360) % 360;
        double errorShortest = ((target - position + 540) % 360) - 180;

        double error= errorShortest;
        if (error < 25) {
            error = errorShortest;   // allow small backward correction
        }else{
                error = errorForward;    // otherwise always go forward
            }

            if (Math.abs(error) < 1) {
                lastError = 0;
                integralSum = 0;
                spindex.setPower(0);

            } else {


// PID
                integralSum += error * dt;
                integralSum = Math.max(-50, Math.min(50, integralSum));

                double derivative = (error - lastError) / dt;
                double ff = KfSpindex * Math.signum(error);
                double output = KpSpindex * error + KiSpindex * integralSum + KdSpindex * derivative + ff;


// Cap speed
                output = Math.max(-1, Math.min(1, output));
                spindex.setPower(output);

                lastError = error;
            }


    }
    public void resetSpindexEncoder(){
        spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void spindexIntake(){
        goToPosition(47.5);
    }
    public void runSpindexToggle(double power){
        if (runTimer.seconds()>0.05){
            spindex.setPower(0);
            runTimer.reset();
        } else {
            spindex.setPower(power);
        }
    }
    public void runSpindexToggleAuto(double power){
        if (runTimer.seconds()>0.1){
            spindex.setPower(power);
            runTimer.reset();
        } else {
            spindex.setPower(0);
        }
    }


    }

