package pathplanner.preprocessor.boundssolver;

import java.awt.geom.Path2D;
import java.awt.geom.Rectangle2D;
import java.util.List;
import java.util.Set;

import pathplanner.common.Obstacle2D;
import pathplanner.common.Vector2D;


public class BoundsSolverData {
  public final Set<Obstacle2D> inActiveObstacles;
  public final List<Vector2D> requiredPoints;
  public final List<Rectangle2D> requiredRects;
  public final Path2D searchArea;
  
  public BoundsSolverData(Set<Obstacle2D> inActiveObstacles, List<Vector2D> requiredPoints, List<Rectangle2D> requiredRects, Path2D searchArea){
      this.inActiveObstacles = inActiveObstacles;
      this.requiredPoints = requiredPoints;
      this.requiredRects = requiredRects;
      this.searchArea = searchArea;
  }
}
