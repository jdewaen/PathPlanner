package pathplanner.preprocessor;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import javafx.util.Pair;
import pathplanner.common.Region2D;


public class CornerEvent implements Comparable<CornerEvent>{
    public final Node start;
    public final Node end;
    
    
    
    public CornerEvent(Node start, Node end){
        this.start = start;
        this.end = end;
    }
    
    public CornerEvent(Node single){
        this.start = single;
        this.end = single;
    }

    @Override
    public int compareTo(CornerEvent o) {
        return Double.compare(start.cost, o.start.cost);
    }
    
    public boolean overlaps(CornerEvent o) {
        if(this.start.cost >= o.start.cost && this.start.cost <= o.end.cost) return true;
        if(o.start.cost >= this.start.cost && o.start.cost <= this.end.cost) return true;
        return false;
    }
    
    public CornerEvent merge(CornerEvent o){
        Node startNode;
        Node endNode;
        
        if( start.cost < o.start.cost){
            startNode = start;
        }else{
            startNode = o.start;
        }
        
        if( end.cost > o.end.cost){
            endNode = end;
        }else{
            endNode = o.end;
        }
        
        return new CornerEvent(startNode, endNode);
        
    }
    
    // ASSUMPTION: each obstacle is only in one event (--> no repeated passes)
    public static List<CornerEvent> generateEvents(List<Pair<Node, Region2D>> nodes, double maxDeltaCost){
        List<CornerEvent> result = new ArrayList<CornerEvent>();
        
        Map<Region2D, List<Node>> eventNodes = new HashMap<Region2D, List<Node>>();
        
        
        for(Pair<Node, Region2D> pair : nodes){
            Node node = pair.getKey();
            Region2D region = pair.getValue();
            
            if(! eventNodes.containsKey(region)) eventNodes.put(region, new ArrayList<Node>());
            eventNodes.get(region).add(node);
        }
        
        
        List<CornerEvent> tempEvents = new ArrayList<CornerEvent>();
        for(Region2D region : eventNodes.keySet()){
            List<Node> currentNodes = eventNodes.get(region);
            Collections.sort(currentNodes);
            
            Node startNode = currentNodes.get(0);
            Node lastNode = currentNodes.get(0);
            double lastCost = startNode.cost;
            int i = 1;
            while(i < currentNodes.size()){
                Node currentNode = currentNodes.get(i);
                if(currentNode.cost > lastCost + maxDeltaCost){
                    CornerEvent event = new CornerEvent(startNode, lastNode);
                    tempEvents.add(event);
                    startNode = currentNode;
                }
                i++;
                lastNode = currentNode;
                lastCost = currentNode.cost;
            }
            
            CornerEvent event = new CornerEvent(startNode, currentNodes.get(currentNodes.size()-1));
            tempEvents.add(event);
        }
        
        Collections.sort(tempEvents);
        
        CornerEvent lastEvent = tempEvents.get(0);
        for( int i = 1; i < tempEvents.size(); i++){
            CornerEvent currentEvent = tempEvents.get(i);
            
            if(lastEvent.overlaps(currentEvent)){
                lastEvent = lastEvent.merge(currentEvent);
            }else{
                result.add(lastEvent);
                lastEvent = currentEvent;
            }
        }
        result.add(lastEvent);
        Collections.sort(result);
        return result;
        
    }


}
