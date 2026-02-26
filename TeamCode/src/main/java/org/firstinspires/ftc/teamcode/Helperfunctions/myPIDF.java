package org.firstinspires.ftc.teamcode.Helperfunctions;

import static org.firstinspires.ftc.teamcode.myConstants.Turret.KdTurret;
import static org.firstinspires.ftc.teamcode.myConstants.Turret.KiTurret;
import static org.firstinspires.ftc.teamcode.myConstants.Turret.KpTurret;
import static org.firstinspires.ftc.teamcode.myConstants.Turret.TURRET_MAX;
import static org.firstinspires.ftc.teamcode.myConstants.Turret.TURRET_MIN;

import com.acmerobotics.dashboard.config.Config;

@Config

public class myPIDF {
    double P;
    double I;
    double D;
    double F;
    double lastError;
    double integralSum;
    double max, min;
    double marginOfError;
    double currentPos, targetPos;
    private final double dt = 0.02;

    public void setMarginOfError(double marg){
        marginOfError = marg;
    }
    public void setMax(double max1){
        max = max1;
    }
    public void setMin(double min1){
        min = min1;
    }


    public void setP(double p1){
        P = p1;

    }
    public void setD(double d1){
        D=d1;
    }
    public void setI (double i1){
        I = i1;
    }
    public void setF (double f1){
        F = f1;
    }

    public myPIDF (double p, double i, double d, double f, double Max, double Min, double marginOfError1, double targetPos1){
        P=p;
        I=i;
        D=d;
        F=f;
        max = Max;
        min = Min;
        integralSum = 0;
        lastError = 0;
        marginOfError = marginOfError1;
        targetPos = targetPos1;

    }

    public myPIDF (double p, double i, double d, double f, double Max, double Min, double marginOfError1){
        P=p;
        I=i;
        D=d;
        F=f;
        max = Max;
        min = Min;
        integralSum = 0;
        lastError = 0;
        marginOfError = marginOfError1;
        targetPos = 0;

    }
    public double calculatePIDF (Double currentPos1, Double targetPos1){
        currentPos = currentPos1;
        targetPos = targetPos1;

        double error = targetPos - currentPos;


        if ( Math.abs(error)<1) {
            lastError = 0;
            integralSum =0;
            return 0;

        } else {



// PID
            integralSum += error * dt;
            integralSum = Math.max(-50, Math.min(50, integralSum));

            double derivative = (error - lastError) / dt;
            double ff = F * Math.signum(error);
            double output = P*error + I*integralSum + D*derivative+ff;


// Cap speed
            output = Math.max(min, Math.min(max, output));

            lastError = error;
            return output;


        }
    }
    public double calculatePIDF ( double p1, double i1, double d1, double f1, Double currentPos1, Double targetPos1){
        P = p1;
        I =i1;
        D=d1;
        F = f1;

        currentPos = currentPos1;
        targetPos = targetPos1;

        double error = targetPos - currentPos;


        if ( Math.abs(error)<1) {
            lastError = 0;
            integralSum =0;
            return 0;

        } else {



// PID
            integralSum += error * dt;
            integralSum = Math.max(-50, Math.min(50, integralSum));

            double derivative = (error - lastError) / dt;
            double output = KpTurret*error + KiTurret*integralSum + KdTurret*derivative;


// Cap speed
            output = Math.max(min, Math.min(max, output));

            lastError = error;
            return output;


        }
    }
     public double calculatePIDF (Double currentPos){

        double error = targetPos - currentPos;


         if ( Math.abs(error)<1) {
             lastError = 0;
             integralSum =0;
             return 0;

         } else {



// PID
             integralSum += error * dt;
             integralSum = Math.max(-50, Math.min(50, integralSum));

             double derivative = (error - lastError) / dt;
             double output = KpTurret*error + KiTurret*integralSum + KdTurret*derivative;


// Cap speed
             output = Math.max(min, Math.min(max, output));

             lastError = error;
             return output;


         }
     }
     public void updatePIDF(double p, double i, double d, double f){
        P=p;
        I=i;
        D=d;
        F=f;

     }


}
