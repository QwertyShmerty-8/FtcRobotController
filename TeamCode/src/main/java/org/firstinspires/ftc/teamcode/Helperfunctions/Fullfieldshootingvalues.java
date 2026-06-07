package org.firstinspires.ftc.teamcode.Helperfunctions;

import com.arcrobotics.ftclib.util.InterpLUT;

public class Fullfieldshootingvalues {
    InterpLUT Hoodlut = new InterpLUT();
    InterpLUT  Flywheellut = new InterpLUT();
    InterpLUT  DistanceTimelut = new InterpLUT();
    Boolean isBlue;
    double firstNumber;
    double lastNumber;

    public Fullfieldshootingvalues(String goalColor){
        isBlue = goalColor.equalsIgnoreCase("blue");


        //Hood Angle Data (Distance, Hood Angle)
        firstNumber = 43.5;
        Hoodlut.add(43.5,0.87);
        Hoodlut.add(53.5, 0.59);
        Hoodlut.add(63.5, 0.45);
        Hoodlut.add(73.5, 0.38);
        Hoodlut.add(83.5, 0.25);
        Hoodlut.add(93.5,0.15);
        Hoodlut.add(138,0.15);
        Hoodlut.add(144,0.15);
        Hoodlut.add(151,0.15);





        //FlywheelSpeed Data (Distance, FlywheelSpeed)
        Flywheellut.add(43.5,-1000);
        Flywheellut.add(53.5, -1100);
        Flywheellut.add(63.5, -1100);
        Flywheellut.add(73.5, -1200);
        Flywheellut.add(83.5, -1300);
        Flywheellut.add(93.5,-1300);
        Flywheellut.add(138,-1480);
        Flywheellut.add(144,-1520);
        Flywheellut.add(151,-1520);
        lastNumber = 151;




        Hoodlut.createLUT();
        Flywheellut.createLUT();
    }

    public double flywheelspeedlut(double x, double y){
        double distance;
        if (isBlue==true){
             distance = Math.sqrt(x-8*x-8 +(136-y)*(136-y));
        }else{
            distance = Math.sqrt((131-x)*(131-x) +(137-y)*(137-y));
        }
        if (distance>lastNumber){
            return -1200;
        }else if(distance<firstNumber){
            return -1200;

        }else {
            return Flywheellut.get(distance);
        }


    }
    public double flywheelspeedlut(double distance){

        if (distance>lastNumber){
            return -1520;
        }else if(distance<firstNumber){
            return -1060;

        }else {
            return Flywheellut.get(distance);
        }


    }


    public double hoodanglelut(double x, double y) {
        double distance;
        if (isBlue == true) {
            distance = Math.sqrt(x * x + (144 - y) * (144 - y));
        } else {
            distance = Math.sqrt((144 - x) * (144 - x) + (144 - y) * (144 - y));
        }
        if (distance > lastNumber) {
            return 1;
        } else if (distance < firstNumber) {
            return 0;

        } else {
            return Hoodlut.get(distance);
        }
    }

    public double hoodanglelut(double distancex) {
        double distance=distancex;

        if (distance > lastNumber) {
            return 1;
        } else if (distance < firstNumber) {
            return 0;

        } else {
            return Hoodlut.get(distance);
        }
    }

        public double distancetimelut ( double x, double y){

            double distance;
            if (isBlue == true) {
                distance = Math.sqrt(x * x + (144 - y) * (144 - y));
            } else {
                distance = Math.sqrt((144 - x) * (144 - x) + (144 - y) * (144 - y));
            }
            if (distance > lastNumber) {
                return 1;
            } else if (distance < firstNumber) {
                return 0;

            } else {
                return DistanceTimelut.get(distance);
            }

        }

    public double getDistance(double x, double y){
        if (isBlue==true){
             return Math.sqrt(x*x +(144-y)*(144-y));
        }else{
            return Math.sqrt((144-x)*(144-x) +(144-y)*(144-y));
        }

    }






}
