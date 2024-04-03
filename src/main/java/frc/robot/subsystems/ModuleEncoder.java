package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;

public class ModuleEncoder {
    private Encoder encoder;
    private boolean inverted = false;
    public ModuleEncoder(int a, int b) {
        this.encoder = new Encoder(a, b);
        encoder.setDistancePerPulse(2.0/8192.0);
    }

    public ModuleEncoder(int a, int b, boolean inverted) {
        this.encoder = new Encoder(a, b);
        encoder.setDistancePerPulse(2.0/8192.0);
        this.inverted = inverted;
        encoder.setReverseDirection(inverted);
    }

    /**
     * degrees not normalized
     * @return
     */
    public double getRawDegrees() {
        return Units.radiansToDegrees(getRawRadians());
    }

    public double getRawRadians() {
        return encoder.getDistance()*2.0*Math.PI;
    }

    public double getNormalizedAngle() {
        return Math.IEEEremainder(getRawRadians(), 2*Math.PI);
    }

    public Rotation2d getRawRotation2d() {
        return Rotation2d.fromRadians(getRawRadians());
    }

    /**
     * 
     * @return rot per sec
     */
    public double getRate() {
        return encoder.getRate();
    }
    
}
