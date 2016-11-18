package pathplanner.milpplanner;

import pathplanner.common.*;


public abstract class Line implements ObstacleConstraint{
    
    
    public static Line fromRegion(Region2D region, Pos2D p1, Pos2D p2) throws Exception{
        
        boolean hori = (p1.x > region.topLeftCorner.x && p2.x > region.topLeftCorner.x) 
                || (p1.x < region.bottomRightCorner.x && p2.x < region.bottomRightCorner.x);
        
        boolean veri = (p1.y > region.topLeftCorner.y && p2.y > region.topLeftCorner.y)  
                || (p1.y < region.bottomRightCorner.y && p2.y < region.bottomRightCorner.y);
        
        if(hori && veri) return null;
        if(!hori && !veri) throw new Exception("Path intersects obstacle!");
        
        
        if(hori){
            if(p1.y < region.topLeftCorner.y){
                return new RegularLine(0, region.topLeftCorner.y, false);
            }else if(p1.y > region.bottomRightCorner.y){
                return new RegularLine(0, region.bottomRightCorner.y, true);
            }else{
                throw new Exception("Point is inside obstacle");
            }
        }else{
            if(p1.x < region.topLeftCorner.x){
                return new VerticalLine(region.topLeftCorner.x, true);
            }else if(p1.y > region.bottomRightCorner.y){
                return new VerticalLine(region.bottomRightCorner.x, false);
            }else{
                throw new Exception("Point is inside obstacle");
            }
        }
        
    }

}
