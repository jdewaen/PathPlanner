package pathplanner.preprocessor.boundssolver;

import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;

import pathplanner.common.Pos2D;


public class BoundsSolverDebugData {
    public List<Pos2D> requiredPoints = new ArrayList<Pos2D>();
    public List<Rectangle2D> requiredRects = new ArrayList<Rectangle2D>();
    public List<Pos2D> seed = new ArrayList<Pos2D>();
}
