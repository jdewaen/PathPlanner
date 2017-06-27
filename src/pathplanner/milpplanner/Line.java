package pathplanner.milpplanner;

import java.io.Serializable;

import pathplanner.common.Vector2D;
import pathplanner.common.Vehicle;
import pathplanner.preprocessor.PathSegment;


public abstract class Line implements ObstacleConstraint, Serializable{
    
/**
     * 
     */
    private static final long serialVersionUID = 1948448751388891654L;
//    public static double LARGE_NUM = 1000000000; // a million kilometers
    public static double LARGE_NUM = 0;

    
    
    
    public static Line fromFinish(PathSegment segment, Vehicle vehicle){
        Vector2D pos = segment.end.pos;
        Vector2D delta = segment.getFinishVector();
        pos = pos.minus(delta.multiply(vehicle.size));
        if(delta.y == 0){
            return new VerticalLine(pos.x, (delta.x < 0));
        }else{
            double a = - delta.x / delta.y;
            double b = pos.y - a * pos.x;
            return new RegularLine(a, b, (delta.y > 0));
        }
    }

}
