package pathplanner.preprocessor.segments;


public class PathSegmentGeneratorConfig {
    public final double approachMargin;
    public final double maxSegmentTime;
    public final boolean verbose;
    
    public PathSegmentGeneratorConfig(double approachMargin, double maxSegmentTime, boolean verbose){
        this.approachMargin = approachMargin;
        this.maxSegmentTime = maxSegmentTime;
        this.verbose = verbose;
    }
}
