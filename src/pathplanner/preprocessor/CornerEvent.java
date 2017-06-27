package pathplanner.preprocessor;

import java.io.Serializable;




public class CornerEvent implements Comparable<CornerEvent>, Serializable{
    /**
     * 
     */
    private static final long serialVersionUID = -940151838889400438L;
    public final PathNode start;
    public final PathNode end;    
    
    public CornerEvent(PathNode start, PathNode end){
        this.start = start;
        this.end = end;
    }

    @Override
    public int compareTo(CornerEvent o) {
        int startComp = Double.compare(start.distance, o.start.distance);
        if(startComp == 0){
            return Double.compare(end.distance, o.end.distance);
        }else{
            return startComp;
        }
    }
    
    public boolean overlaps(CornerEvent o) {
        if(this.start.distance >= o.start.distance && this.start.distance <= o.end.distance) return true;
        if(o.start.distance >= this.start.distance && o.start.distance <= this.end.distance) return true;
        return false;
    }

}
