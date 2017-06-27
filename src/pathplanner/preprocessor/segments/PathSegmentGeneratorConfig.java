package pathplanner.preprocessor.segments;

import java.io.Serializable;


public class PathSegmentGeneratorConfig implements Serializable{
    /**
     * 
     */
    private static final long serialVersionUID = 5753106039899668346L;
    public final double approachMargin;
    public final double maxSegmentTime;
    public final boolean verbose;
    
    public PathSegmentGeneratorConfig(double approachMargin, double maxSegmentTime, boolean verbose){
        this.approachMargin = approachMargin;
        this.maxSegmentTime = maxSegmentTime;
        this.verbose = verbose;
    }
}
