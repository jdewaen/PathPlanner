package pathplanner.milpplanner;

import pathplanner.common.*;


public abstract class Line implements ObstacleConstraint{
    
    
    public static Line fromRegion(Region2D region, Pos2D point, boolean horizontal) throws Exception{
        if(horizontal){
            if(point.y < region.bottomRightCorner.y){
                return new RegularLine(0, region.bottomRightCorner.y, false);
            }else if(point.y > region.topLeftCorner.y){
                return new RegularLine(0, region.topLeftCorner.y, true);
            }else{
                throw new Exception("Point is inside obstacle");
            }
        }else{
            if(point.x < region.bottomRightCorner.x){
                return new VerticalLine(region.bottomRightCorner.x, true);
            }else if(point.y > region.topLeftCorner.y){
                return new VerticalLine(region.topLeftCorner.x, false);
            }else{
                throw new Exception("Point is inside obstacle");
            }
        }
        
    }

}
