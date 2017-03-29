package pathplanner.preprocessor;


public enum Neighbor{
    UP, DOWN, LEFT, RIGHT;
    
    public boolean horizontalEdge;
    public boolean verticalEdge;
    public boolean minEdge;
    public boolean maxEdge;
    static {
        UP.horizontalEdge = true;
        DOWN.horizontalEdge = true;
        LEFT.horizontalEdge = false;
        RIGHT.horizontalEdge = false;
        UP.verticalEdge = !UP.horizontalEdge;
        DOWN.verticalEdge = !DOWN.horizontalEdge;
        LEFT.verticalEdge = !LEFT.horizontalEdge;
        RIGHT.verticalEdge = !RIGHT.horizontalEdge;
        
        UP.minEdge = false;
        DOWN.minEdge = true;
        LEFT.minEdge = true;
        RIGHT.minEdge = false;
        
        UP.maxEdge = !UP.minEdge;
        DOWN.maxEdge = !DOWN.minEdge;
        LEFT.maxEdge = !LEFT.minEdge;
        RIGHT.maxEdge = !RIGHT.minEdge;
    }

}