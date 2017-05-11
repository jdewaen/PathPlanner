package pathplanner.preprocessor.segments;


public class SegmentGeneratorConfig {
    public final double approachMargin;
    public final double maxSegmentTime;
    public final boolean verbose;
    
    public SegmentGeneratorConfig(double approachMargin, double maxSegmentTime, boolean verbose){
        this.approachMargin = approachMargin;
        this.maxSegmentTime = maxSegmentTime;
        this.verbose = verbose;
    }
}
