package pathplanner.milpplanner;

import java.util.HashMap;
import java.util.List;

import pathplanner.common.Region2D;
import ilog.concert.*;

public class SolutionVars {
    public IloNumVar[] posX;
    public IloNumVar[] posY;
    
    public IloNumVar[] velX;
    public IloNumVar[] velY;
    
    public IloNumVar[] verticalThrottle;
    public IloNumVar[] horizontalThrottle;
    
    public IloNumVar[] fin;
    public IloNumVar[] cfin;
    
    public IloNumVar[] time;
    
    public HashMap<Region2D, IloNumVar> checkpoints;
    public HashMap<Region2D, IloIntVar[]> checkpointsCounter;
    public HashMap<Region2D, IloIntVar[]> checkpointsChange;
    
//    public final int size;
    
//    public SolutionVars(int size){
//        this.size = size;
//        posX = new IloNumVar[size];
//        posY = new IloNumVar[size];
//        
//        velX = new IloNumVar[size];
//        velY = new IloNumVar[size];
//        
//        verticalThrottle = new IloNumVar[size];
//        horizontalThrottle = new IloNumVar[size];
//        
//        fin = new IloNumVar[size];
//        cfin = new IloNumVar[size];
//        
//        time = new IloNumVar[size];
//        
//        checkpoints = new HashMap<Region2D, IloNumVar>();
//        checkpointsCounter = new HashMap<Region2D, IloIntVar[]>();
//        checkpointsChange = new HashMap<Region2D, IloIntVar[]>();
//    }
//    
//    
//    
//    public static SolutionVars combine(List<SolutionVars> list){
//        int sum = 0;
//        for(SolutionVars sol :  list){
//            sum += sol.size;
//        }
//        
//        SolutionVars result = new SolutionVars(sum);
//        
//        
//        int counter = 0;
//        for(SolutionVars sol :  list){
//            for(int i = 0; i < sol.size; i++){
//                result.posX[counter] = sol.posX[i];
//                result.posY[counter] = sol.posY[i];
//                
//                result.velX[counter] = sol.velX[i];
//                result.velY[counter] = sol.velY[i];
//                
//                result.verticalThrottle[counter] = sol.verticalThrottle[i];
//                result.horizontalThrottle[counter] = sol.horizontalThrottle[i];
//                
//                result.fin[counter] = sol.fin[i];
//                result.cfin[counter] = sol.cfin[i];
//                
//                result.time[counter] = sol.time[i];
//                                
//                counter++;
//            }
//        }
//    
//        return result;
//    }
    

    

}
