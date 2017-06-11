package pathplanner.preprocessor.boundssolver;

import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;

import pathplanner.common.Vector2D;


public class BoundsSolverDebugData {
    public List<Vector2D> requiredPoints = new ArrayList<Vector2D>();
    public List<Rectangle2D> requiredRects = new ArrayList<Rectangle2D>();
    public List<Vector2D> seed = new ArrayList<Vector2D>();
}
