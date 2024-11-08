package frc.robot;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class IntakeConstants { // TODO: Update Intake CAN IDs
        public static final int intakeMotorOneID = 13;
        public static final double intakeSpeed = 0.6;
    }

    public static final class ElevatorConstants { // TODO: Update Elevator CAN IDs
        public static final int elevatorMotorID = 16;
        public static final int elevatorTopLimitChannel = 5;
        public static final int elevatorBottomLimitChannel = 6;
        public static final double elevatorSpeed = .5;
    }

    public static final class IndexerConstants { // TODO: Update Indexer CAN IDs and Channels
        public static final int indexMotorID = 32;
        public static final int indexBeamBreakChannel = 61;
        public static final double indexSpeed = 0.23;
        public static final double beamBreakRange = 200;
    }

    public static final class ShooterConstants {
        public static final int shooterMotorOneID = 21;
        public static final int shooterMotorTwoID = 33;
        public static final double distanceShotSpeed = 0.5;
        public static final double ampShotSpeed = .4;
        public static final double shortShotSpeed = .6;
    }

    public static final class WristConstants {
        public static final int wristMotorID = 15;
        public static final int wristLimitOneChannel = 9;
        public static final int wristLimitTwoChannel = 8;
        public static final int subwofferAngle = 75;
        public static final int ampAngle = 85;
        public static final int distanceAngle = 60;
        public static final double wristP = 0;
        public static final double wristI = 0;
        public static final double wristD = 0;
        public static final double wristPosTolerance = 1;
        public static final double wristVelTolerance = 1;
    }
    public static final class LEDConstants {
        public static final int LEDDriverOneID = 3;
        public static final double colorRed = 0.61;
        public static final double colorHotPink = 0.57;
        public static final double colorYellow = 0.69;
        public static final double colorSkyBlue = 0.83;
        public static final double colorBlueViolet = 0.89;
        public static final double colorWhite = 0.93;
        public static final double colorLimeGreen = 0.73;
        public static final double colorOrange = 0.65;
        public static final double colorDarkGreen = 0.75;
        public static final double colorLawnGreen = 0.71;
        public static final double colorBlue = 0.87;
        public static final double colorGold = 0.67;
        public static final double twinklesColorOneAndTwo = 0.51;
    }
}