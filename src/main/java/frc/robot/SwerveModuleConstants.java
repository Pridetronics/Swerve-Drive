package frc.robot;

public class SwerveModuleConstants {
    public final int kDriveMotorCANID;
    public final int kTurningMotorCANID;
    public final int kTurningEncoderID;
    public final double kAbsoluteEncoderOffsetRadians;
    public final boolean kAbsoluteEncoderReversed;
    public final boolean kDriveEncoderReversed;
    public final boolean kTurningEncoderReversed;

    SwerveModuleConstants(
            int driveMotorCANID, 
            int turningMotorCANID, 
            int CTRETurningEncoderID,
            int absoluteEncoderOffsetDegrees,
            boolean absoluteEncoderReversed,
            boolean driveEncoderReversed,
            boolean turningEncoderReversed
        ) {

        kDriveMotorCANID = driveMotorCANID;
        kTurningMotorCANID = turningMotorCANID;
        kTurningEncoderID = CTRETurningEncoderID;
        kAbsoluteEncoderOffsetRadians = absoluteEncoderOffsetDegrees * (Math.PI/180);
        kAbsoluteEncoderReversed = absoluteEncoderReversed;
        kDriveEncoderReversed = driveEncoderReversed;
        kTurningEncoderReversed = turningEncoderReversed;
        
    }
}
