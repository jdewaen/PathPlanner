package pathplanner.preprocessor.segments;

import pathplanner.preprocessor.cornerheuristic.ThetaStarConfig;



public class SegmentGeneratorConfigFactory {
    public double approachMargin = 2;
    public double maxSegmentTime = 3;
    public boolean verbose = false;

    public static final SegmentGeneratorConfig DEFAULT = (new SegmentGeneratorConfigFactory()).build();
    
    public SegmentGeneratorConfig build(){
        return new SegmentGeneratorConfig(
                approachMargin,
                maxSegmentTime,
                verbose);
    }
}
