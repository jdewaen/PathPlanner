package pathplanner.milpplanner;

import pathplanner.common.Pos2D;
import pathplanner.preprocessor.PathSegment;


public abstract class Line implements ObstacleConstraint{
    
//    public static double LARGE_NUM = 1000000000; // a million kilometers
    public static double LARGE_NUM = 0;

    
    
    
    public static Line fromFinish(PathSegment segment){
        Pos2D pos = segment.end.pos;
        Pos2D delta = segment.getFinishVector();
        if(delta.y == 0){
            return new VerticalLine(pos.x, (delta.x < 0));
        }else{
            double a = - delta.x / delta.y;
            double b = pos.y - a * pos.x;
            return new RegularLine(a, b, (delta.y > 0));
        }
    }

}
