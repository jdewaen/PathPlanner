package pathplanner.preprocessor.cornerheuristic;




public class ThetaStarConfigFactory {
    public double gridSize = 2;
    public double tolerance = 2;
    public boolean verbose = false;

    public static final ThetaStarConfig DEFAULT = (new ThetaStarConfigFactory()).build();
    
    public ThetaStarConfig build(){
        return new ThetaStarConfig(
                gridSize,
                tolerance,
                verbose);
    }
}
