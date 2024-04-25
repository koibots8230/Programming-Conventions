package frc.lib.util;

public class FeedForwardConstants {

    public final double kS;
    public final double kV;
    public final double kA;
    public final double kG;

    public FeedForwardConstants(double kS, double kV) {
        this.kS = kS;
        this.kV = kV;
        this.kA = 0;
        this.kG = 0;
    }

    public FeedForwardConstants(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = 0;
    }

    public FeedForwardConstants(double kS, double kV, double kA, double kG) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
    }
}
