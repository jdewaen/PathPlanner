package pathplanner.preprocessor.segments;


public class SegmentGeneratorConfig {
    public final double approachMargin;
    public final double maxSegmentTime;
    public static final SegmentGeneratorConfig DEFAULT = new SegmentGeneratorConfig(2, 5);
    
    public SegmentGeneratorConfig(double approachMargin, double maxSegmentTime){
        this.approachMargin = approachMargin;
        this.maxSegmentTime = maxSegmentTime;
    }
}
