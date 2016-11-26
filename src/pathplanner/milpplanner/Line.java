package pathplanner.milpplanner;

import pathplanner.common.Pos2D;
import pathplanner.common.Region2D;
import pathplanner.preprocessor.Node;


public abstract class Line implements ObstacleConstraint{
    
    
    public static Line fromRegion(Region2D region, Pos2D p1, Pos2D p2) throws Exception{
        
        boolean hori = (p1.x > region.topLeftCorner.x && p2.x > region.topLeftCorner.x) 
                || (p1.x < region.bottomRightCorner.x && p2.x < region.bottomRightCorner.x);
        
        boolean veri = (p1.y > region.topLeftCorner.y && p2.y > region.topLeftCorner.y)  
                || (p1.y < region.bottomRightCorner.y && p2.y < region.bottomRightCorner.y);
        
        if(hori && veri) return null;
        if(!hori && !veri) throw new Exception("Path intersects obstacle!");
        
        
        if(veri){
            if(p1.y < region.bottomRightCorner.y){
                return new RegularLine(0, region.bottomRightCorner.y, false);
            }else if(p1.y > region.topLeftCorner.y){
                return new RegularLine(0, region.topLeftCorner.y, true);
            }else{
                throw new Exception("Point is inside obstacle");
            }
        }else{
            if(p1.x < region.bottomRightCorner.x){
                return new VerticalLine(region.bottomRightCorner.x, true);
            }else if(p1.x > region.topLeftCorner.x){
                return new VerticalLine(region.topLeftCorner.x, false);
            }else{
                throw new Exception("Point is inside obstacle");
            }
        }
        
    }
    
    
    public static Line fromFinish(Pos2D pos, Node last){
        Pos2D delta = last.pos.minus(last.parent.pos);
        if(delta.y == 0){
            return new VerticalLine(pos.x, (delta.x < 0));
        }else if(delta.x == 0){
            return new RegularLine(0, pos.y, (delta.y > 0));
        }else{
            double a = - delta.x / delta.y;
            double b = pos.y - a * pos.x;
            return new RegularLine(a, b, above)????
        }
    }

}
