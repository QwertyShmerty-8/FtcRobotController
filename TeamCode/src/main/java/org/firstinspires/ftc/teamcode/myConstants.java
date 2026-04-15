package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;


public class myConstants {

    @Config
    public static class intake {
        public static double intakeLIntakePos = 1;
        public static double intakeRIntakePos = 0;
        public static double intakeLoutakePos = 0.65;
        public static double intakeRoutakePos = 0.35;
        public static double intakeLSwitchBallPos = 0.75;
        public static double intakeRSwitchBallPos = 0.25;

    }
    @Config
    public static class Turret{

        public static double Pflywheel = 24;
        public static double Fflywheel = 15;
        public static double Iflywheel=0.5;
        public static double  Dflywheel=0.5;
       public static double TURRET_MIN = -170;
        public static double TURRET_MAX =  170;
    }
    @Config
    public static class Spindex{


    }
}
