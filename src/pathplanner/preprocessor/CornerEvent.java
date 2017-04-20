package pathplanner.preprocessor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import javafx.util.Pair;
import pathplanner.common.Obstacle2DB;


public class CornerEvent implements Comparable<CornerEvent>{
    public final Node start;
    public final Node end;
    public final Set<Obstacle2DB> regions = new HashSet<Obstacle2DB>();
    public final List<CornerEvent> parents = new ArrayList<CornerEvent>();
    
    
    
//    public CornerEvent(Node start, Node end, Obstacle2DB region, double expansionDist, List<Node> list){
////        this.start = start;
////        this.end = end;
//        
//        double goalFirst = start.cost - expansionDist;
//        Node currentFirst = start;
//        while(currentFirst.cost > goalFirst && currentFirst.parent != null){
//            currentFirst = currentFirst.parent;
//        }
//        this.start = currentFirst;
//        
//        
//        double goalLast = end.cost + expansionDist;
//        Node currentLast = start;
//        while(currentLast.cost < goalLast && !currentLast.children.isEmpty()){
//            currentLast = currentLast.getChild();
//        }
//        this.end = currentLast;     
//        
//        this.regions.add(region);
//    }
    
    public CornerEvent(Node start, Node end, Set<Obstacle2DB> regions){
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
    
//    public static List<CornerEvent> generateEvents2(Map<Node, Set<Obstacle2DB>> nodes, double maxDeltaCost, Node start){
//        List<CornerEvent> result = new ArrayList<CornerEvent>();
//        Node current = start;
//        while(current != null){
//            if(!nodes.containsKey(current)){
//                current = current.getChild();
//                continue;
//            }
//            
//            Node lastNodeOfCorner = current;
//            int turnDirection = lastNodeOfCorner.getTurnDirection();
//            if(turnDirection == 0){
//                System.out.println("NO TURN AT: " + lastNodeOfCorner.pos.toPrettyString());
//                current = current.getChild();
//                continue;
//            }
//            Node currentCornerNode = lastNodeOfCorner.getChild();
//            while(currentCornerNode != null){
//                if(nodes.containsKey(currentCornerNode)){
//                    if(currentCornerNode.getTurnDirection() != -turnDirection){
//                        break;
//                        // DIRECTION CHANGED!!! MAKE EVENT AND KEEP GOING HERE
//                    }else{
//                        lastNodeOfCorner = currentCornerNode;
//                        currentCornerNode = currentCornerNode.getChild();
//                        continue;
//                        // SAME DIRECTION: UPDATE NODE
//                    }
//                }else{
//                    // Not in the corner list, keep going until maxDeltaCost is reached
//                    if(currentCornerNode.cost - lastNodeOfCorner.cost > maxDeltaCost){
//                        break;
//                    }
//                    currentCornerNode = currentCornerNode.getChild();
//                    continue;
//                }
//            }
//            
//            // Corner is fully expanded
//            CornerEvent event = new CornerEvent(current, lastNodeOfCorner, new HashSet<Obstacle2DB>());
//            current = currentCornerNode;
//            result.add(event);
//        }
//        return result;
//        
//    }
    
    public static List<CornerEvent> generateEvents3(List<Node> nodes, double maxDeltaCost, Node start){
        ArrayList<Node> list = new ArrayList<Node>(nodes);
        List<CornerEvent> result = new ArrayList<CornerEvent>();
        int i = 0;
        
        while(i < list.size() - 1){
            Node current = list.get(i);

            Node lastNodeOfCorner = current;
            Node currentCornerNode = list.get(i+1);
            int turnDirection = lastNodeOfCorner.getTurnDirection(currentCornerNode);
            if(turnDirection == 0){
                System.out.println("NO TURN AT: " + lastNodeOfCorner.pos.toPrettyString());
                i++;
                continue;
            }
            i++;
            while(i < list.size() - 1){
                currentCornerNode = list.get(i);
                Node nextNode = list.get(i+1);
                if(currentCornerNode.cost - lastNodeOfCorner.cost > maxDeltaCost){
                    break;
                }
                if(currentCornerNode.getTurnDirection(nextNode) != turnDirection){
                    break; //TODO: check distance,  don't break if really close (half? quarter?)
                    // DIRECTION CHANGED!!! MAKE EVENT AND KEEP GOING HERE
                }else{
                    lastNodeOfCorner = currentCornerNode;
                    currentCornerNode = nextNode;
                    i++;
                    continue;
                    // SAME DIRECTION: UPDATE NODE
                }

            }
            
            // Corner is fully expanded
            CornerEvent event = new CornerEvent(current, lastNodeOfCorner, new HashSet<Obstacle2DB>());
            current = currentCornerNode;
            result.add(event);
        }
        return result;
        
    }
    
 
//    public static List<CornerEvent> generateEvents(List<Pair<Node, Obstacle2DB>> nodes, double maxDeltaCost, double expansionDist){
//        List<CornerEvent> result = new ArrayList<CornerEvent>();
//        
//        Map<Obstacle2DB, List<Node>> eventNodes = new HashMap<Obstacle2DB, List<Node>>();
//        
//        
//        for(Pair<Node, Obstacle2DB> pair : nodes){
//            Node node = pair.getKey();
//            Obstacle2DB region = pair.getValue();
//            
//            if(! eventNodes.containsKey(region)) eventNodes.put(region, new ArrayList<Node>());
//            eventNodes.get(region).add(node);
//        }
//        
//        
//        List<CornerEvent> tempEvents = new ArrayList<CornerEvent>();
//        for(Obstacle2DB region : eventNodes.keySet()){
//            List<Node> currentNodes = eventNodes.get(region);
//            Collections.sort(currentNodes);
//            
//            Node startNode = currentNodes.get(0);
//            Node lastNode = currentNodes.get(0);
//            double lastCost = startNode.cost;
//            int i = 1;
//            while(i < currentNodes.size()){
//                Node currentNode = currentNodes.get(i);
//                if(currentNode.cost > lastCost + maxDeltaCost){
//                    tempEvents.addAll(splitIfNeeded(startNode, lastNode, region, expansionDist));
//                    startNode = currentNode;
//                }
//                i++;
//                lastNode = currentNode;
//                lastCost = currentNode.cost;
//            }
//            
//            tempEvents.addAll(splitIfNeeded(startNode, currentNodes.get(currentNodes.size()-1), region, expansionDist));
//        }
//        
//        Collections.sort(tempEvents);
//        if(tempEvents.isEmpty()) return tempEvents;
//        CornerEvent lastEvent = tempEvents.get(0);
//        for( int i = 1; i < tempEvents.size(); i++){
//            CornerEvent currentEvent = tempEvents.get(i);
//            
//            if(lastEvent.overlaps(currentEvent)){
//                
//                CornerEvent merged = lastEvent.merge(currentEvent);
//                boolean foundIntersect = false;
//                for(Obstacle2DB region : merged.regions){
//                    if(!region.intersects(merged.start.pos, merged.end.pos, 0)){
//                        //  region doesn't intersect line between start and end of event
//                        CornerEvent first = new CornerEvent(lastEvent.start, currentEvent.start, lastEvent.regions);
//                        CornerEvent second = new CornerEvent(currentEvent.start, currentEvent.end, currentEvent.regions);
//                        result.add(first);
//                        lastEvent = second;
//                        foundIntersect = true;
//                        break;
//                    }
//                }
//                
//                if(!foundIntersect){
//                    lastEvent = merged;
//                }
//                
//            }else{
//                result.add(lastEvent);
//                lastEvent = currentEvent;
//            }
//            
//
//        }
//        result.add(lastEvent);
//        Collections.sort(result);
//        return result;
//        
//    }
//    
//    private static List<CornerEvent> splitIfNeeded(Node start, Node end, Obstacle2DB region, double expansionDist){
//        List<CornerEvent> result = new ArrayList<CornerEvent>();
//        if(start.cost > end.cost) return result;
//        CornerEvent event = new CornerEvent(start, end, region, expansionDist);
//        if(start == end || end.parent == start){
//            result.add(event);
//            return result;
//        }
//        if(!region.intersects(event.start.pos, event.end.pos, 0)){
//            Node middle = Node.split(start, end);
//            result.addAll(splitIfNeeded(start, middle, region, expansionDist));
//            result.addAll(splitIfNeeded(middle.getChild(), end, region, expansionDist));
//        }else{
//            result.add(event);
//        }
//        return result;
//    }


}
