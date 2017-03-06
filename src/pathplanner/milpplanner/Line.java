package pathplanner.milpplanner;

import pathplanner.common.Pos2D;
import pathplanner.preprocessor.Node;


public abstract class Line implements ObstacleConstraint{
    
//    public static double LARGE_NUM = 1000000000; // a million kilometers
    public static double LARGE_NUM = 0;

    
    
    
    public static Line fromFinish(Pos2D pos, Node current, int depth){
    	Node last = current;
    	for(int i = 0; i < depth; i++){
    		if(last.parent == null) break;
    		last = last.parent;
    	}
        Pos2D delta = current.pos.minus(last.pos);
        if(delta.y == 0){
            return new VerticalLine(pos.x, (delta.x < 0));
        }else{
            double a = - delta.x / delta.y;
            double b = pos.y - a * pos.x;
            return new RegularLine(a, b, (delta.y > 0));
        }
    }

}
