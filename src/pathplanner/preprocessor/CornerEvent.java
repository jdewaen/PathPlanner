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
    public final Set<Region2D> regions = new HashSet<Region2D>();
    public final List<CornerEvent> parents = new ArrayList<CornerEvent>();
    
    
    
    public CornerEvent(Node start, Node end, Region2D region, double expansionDist){
//        this.start = start;
//        this.end = end;
        
        double goalFirst = start.cost - expansionDist;
        Node currentFirst = start;
        while(currentFirst.cost > goalFirst && currentFirst.parent != null){
            currentFirst = currentFirst.parent;
        }
        this.start = currentFirst;
        
        
        double goalLast = end.cost + expansionDist;
        Node currentLast = start;
        while(currentLast.cost < goalLast && !currentLast.children.isEmpty()){
            currentLast = currentLast.getChild();
        }
        this.end = currentLast;     
        
        this.regions.add(region);
    }
    
    public CornerEvent(Node start, Node end, Set<Region2D> regions){
        this.start = start;
        this.end = end;
        this.regions.addAll(regions);
    }
    
    private CornerEvent(Node start, Node end, CornerEvent parentA, CornerEvent parentB){
        this.start = start;
        this.end = end;
        parents.add(parentA);
        parents.add(parentB);
        regions.addAll(parentA.regions);
        regions.addAll(parentB.regions);
    }

    @Override
    public int compareTo(CornerEvent o) {
        int startComp = Double.compare(start.cost, o.start.cost);
        if(startComp == 0){
            return Double.compare(end.cost, o.end.cost);
        }else{
            return startComp;
        }
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
        
        CornerEvent result = new CornerEvent(startNode, endNode, this, o);
        return result;
        
    }
    
 
    public static List<CornerEvent> generateEvents(List<Pair<Node, Region2D>> nodes, double maxDeltaCost, double expansionDist){
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
                    tempEvents.addAll(splitIfNeeded(startNode, lastNode, region, expansionDist));
                    startNode = currentNode;
                }
                i++;
                lastNode = currentNode;
                lastCost = currentNode.cost;
            }
            
            tempEvents.addAll(splitIfNeeded(startNode, currentNodes.get(currentNodes.size()-1), region, expansionDist));
        }
        
        Collections.sort(tempEvents);
        
        CornerEvent lastEvent = tempEvents.get(0);
        for( int i = 1; i < tempEvents.size(); i++){
            CornerEvent currentEvent = tempEvents.get(i);
            
            if(lastEvent.overlaps(currentEvent)){
                
                CornerEvent merged = lastEvent.merge(currentEvent);
                boolean foundIntersect = false;
                for(Region2D region : merged.regions){
                    if(!region.intersects(merged.start.pos, merged.end.pos)){
                        //  region doesn't intersect line between start and end of event
                        CornerEvent first = new CornerEvent(lastEvent.start, currentEvent.start, lastEvent.regions);
                        CornerEvent second = new CornerEvent(currentEvent.start, currentEvent.end, currentEvent.regions);
                        result.add(first);
                        lastEvent = second;
                        foundIntersect = true;
                        break;
                    }
                }
                
                if(!foundIntersect){
                    lastEvent = merged;
                }
                
            }else{
                result.add(lastEvent);
                lastEvent = currentEvent;
            }
            

        }
        result.add(lastEvent);
        Collections.sort(result);
        return result;
        
    }
    
    private static List<CornerEvent> splitIfNeeded(Node start, Node end, Region2D region, double expansionDist){
        List<CornerEvent> result = new ArrayList<CornerEvent>();
        if(start.cost > end.cost) return result;
        CornerEvent event = new CornerEvent(start, end, region, expansionDist);
        if(start == end || end.parent == start){
            result.add(event);
            return result;
        }
        if(!region.intersects(event.start.pos, event.end.pos)){
            Node middle = Node.split(start, end);
            result.addAll(splitIfNeeded(start, middle, region, expansionDist));
            result.addAll(splitIfNeeded(middle.getChild(), end, region, expansionDist));
        }else{
            result.add(event);
        }
        return result;
    }


}
