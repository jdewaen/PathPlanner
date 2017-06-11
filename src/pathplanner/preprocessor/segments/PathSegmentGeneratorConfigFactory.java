package pathplanner.preprocessor.segments;

import pathplanner.preprocessor.cornerheuristic.ThetaStarConfig;



public class PathSegmentGeneratorConfigFactory {
    public double approachMargin = 2;
    public double maxSegmentTime = 3;
    public boolean verbose = false;

    public static final PathSegmentGeneratorConfig DEFAULT = (new PathSegmentGeneratorConfigFactory()).build();
    
    public PathSegmentGeneratorConfig build(){
        return new PathSegmentGeneratorConfig(
                approachMargin,
                maxSegmentTime,
                verbose);
    }
}
