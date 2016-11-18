package pathplanner.milpplanner;

import java.util.HashMap;

import pathplanner.common.*;
import sun.rmi.transport.proxy.CGIHandler;
import ilog.cplex.*;
import ilog.cplex.IloCplex.UnknownObjectException;
import ilog.concert.*;

public class CPLEXSolver {
    
    static final double FUZZY_DELTA = 0.01;
    static final double FUZZY_DELTA_POS = 0.01;

    private Scenario2D scenario;
    private IloCplex cplex;
    private SolutionVars vars;
    public CPLEXSolver(Scenario2D scenario){
        this.scenario = scenario;
    }
    
    public void generateConstraints(){
        try {
            cplex = new IloCplex();
//            cplex.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, 0.35);
//            cplex.setParam(IloCplex.Param.MIP.Tolerances.AbsMIPGap, 0.5/scenario.deltaT);
            vars = initVars();
            addGoal();
            generateWorldConstraints();
            generateObstacleConstraints();
            generateVehicleConstraints();
//            generateCheckPoints();
            
            
        } catch (IloException e) {
            e.printStackTrace();
        }

    }
    
    private SolutionVars initVars() throws IloException{
        SolutionVars result = new SolutionVars();
        
        result.posX = cplex.numVarArray(scenario.timeSteps, 0, Double.MAX_VALUE);
        result.posY = cplex.numVarArray(scenario.timeSteps, 0, Double.MAX_VALUE);
        cplex.add(result.posX);
        cplex.add(result.posY);

        result.velX = cplex.numVarArray(scenario.timeSteps, -Double.MAX_VALUE, Double.MAX_VALUE);
        result.velY = cplex.numVarArray(scenario.timeSteps, -Double.MAX_VALUE, Double.MAX_VALUE);
        cplex.add(result.velX);
        cplex.add(result.velY);
        
        
        result.horizontalThrottle = cplex.numVarArray(scenario.timeSteps, -1, 1);
        result.verticalThrottle = cplex.numVarArray(scenario.timeSteps, -1, 1);
        cplex.add(result.horizontalThrottle);
        cplex.add(result.verticalThrottle);

        result.fin = cplex.intVarArray(scenario.timeSteps, 0, 1);
        result.cfin = cplex.intVarArray(scenario.timeSteps, 0, 1);
        cplex.add(result.fin);   
        cplex.add(result.cfin);   
        
        result.time = cplex.numVarArray(scenario.timeSteps, 0, Double.MAX_VALUE);
        cplex.add(result.time);

        result.checkpoints = new HashMap<Region2D, IloNumVar>();
        result.checkpointsCounter = new HashMap<Region2D, IloIntVar[]>();
        result.checkpointsChange = new HashMap<Region2D, IloIntVar[]>();

        return result;
    }
    
    private void addGoal() throws IloException{
        cplex.addMinimize(cplex.diff(scenario.timeSteps, cplex.sum(vars.fin)));
        for(int t = 0; t < scenario.timeSteps; t++){
            IloConstraint cfinReq = diff(scenario.goal.x, vars.posX[t], FUZZY_DELTA_POS);
            cfinReq = cplex.and(cfinReq, diff(scenario.goal.y, vars.posY[t], FUZZY_DELTA_POS));
            
            if(scenario.goalVel != null){
                cfinReq = cplex.and(cfinReq, diff(scenario.goalVel.x, vars.velX[t], FUZZY_DELTA));
                cfinReq = cplex.and(cfinReq, diff(scenario.goalVel.y, vars.velY[t], FUZZY_DELTA));
            }

            cplex.add(iff(cfinReq, isTrue(vars.cfin[t])));
            if(t != scenario.timeSteps - 1){
                cplex.add(iff(isTrue(vars.fin[t+1]), cplex.or(isTrue(vars.cfin[t+1]), isTrue(vars.fin[t]))));
            }else{
                cplex.addEq(1, vars.fin[t]);

            }
        }
        
    }
    
   
    private void generateCheckPoints() throws IloException{
        
        for(Region2D obs : scenario.world.getRegions()){
            if(!obs.isCheckPoint()) continue;
            CheckPoint2D cp = (CheckPoint2D) obs;
            int maxTimeStep = (int) (cp.maxTime / scenario.deltaT);
            IloNumVar cpTime = cplex.numVar(0, Double.MAX_VALUE);
            vars.checkpoints.put(cp, cpTime);
            
            IloIntVar[] cpTimes = cplex.intVarArray(scenario.timeSteps, 0, 1);
            IloIntVar[] ccpTimes = cplex.intVarArray(scenario.timeSteps, 0, 1);
            
            vars.checkpointsCounter.put(cp, cpTimes);
            vars.checkpointsChange.put(cp, ccpTimes);
            
            cplex.add(cpTime);
            cplex.add(cpTimes);
            cplex.add(ccpTimes);

            
            cplex.addEq(cpTime, cplex.prod(cplex.diff(scenario.timeSteps, cplex.sum(cpTimes)), scenario.deltaT));
            
            cplex.addLe(cpTime, cp.maxTime);
            
            cplex.addEq(cpTimes[0], 0);
            
            
           
            for(int t = 0; t < maxTimeStep + 1; t++){
//                int largeNum = 99999;
//                IloIntVar[] slack = cplex.intVarArray(4, 0, 1);
//                cplex.addLe(vars.posX[t], cplex.sum(obs.topLeftCorner.x, cplex.prod(largeNum, slack[0])));
//                cplex.addLe(cplex.negative(vars.posX[t]), cplex.sum(-obs.bottomRightCorner.x, cplex.prod(largeNum, slack[1])));
//                cplex.addLe(vars.posY[t], cplex.sum(obs.topLeftCorner.y, cplex.prod(largeNum, slack[2])));
//                cplex.addLe(cplex.negative(vars.posY[t]), cplex.sum(-obs.bottomRightCorner.y, cplex.prod(largeNum, slack[3])));
//                cplex.addLe(cplex.sum(slack), cplex.sum(3, ccpTimes[t]));
                
                
                IloConstraint ccpTimeReq = cplex.ge(vars.posX[t], cp.bottomRightCorner.x);
                ccpTimeReq = cplex.and(ccpTimeReq, cplex.le(vars.posX[t], cp.topLeftCorner.x));
                ccpTimeReq = cplex.and(ccpTimeReq, cplex.ge(vars.posY[t], cp.bottomRightCorner.y));
                ccpTimeReq = cplex.and(ccpTimeReq, cplex.le(vars.posY[t], cp.topLeftCorner.y));
                cplex.add(iff(ccpTimeReq, isTrue(ccpTimes[t])));
                
                if(t != scenario.timeSteps - 1){
                    cplex.add(iff(isTrue(cpTimes[t+1]), cplex.or(isTrue(cpTimes[t]), isTrue(ccpTimes[t]))));
                }else{
                    cplex.add(cpTimes[t]);

                }
                
            }
            
            
        }
        
    }
    
//    private void generateObstacleConstraints() throws IloException{
//        // Obstacles
//        for(Region2D obs : scenario.world.getRegions()){
//            for(int t = (int) (obs.startTime/scenario.deltaT) ; t < Math.min(scenario.timeSteps, obs.endTime / scenario.deltaT); t++){
//                
//                int largeNum = 99999;
//                double buffer = 0;
//                IloIntVar[] slack = cplex.intVarArray(4, 0, 1);;
//                cplex.addLe(cplex.sum(vars.posX[t], buffer), cplex.sum(obs.topLeftCorner.x, cplex.prod(largeNum, slack[0])));
//                cplex.addLe(cplex.negative(cplex.sum(vars.posX[t], -buffer)), cplex.sum(-obs.bottomRightCorner.x, cplex.prod(largeNum, slack[1])));
//                cplex.addLe(cplex.sum(vars.posY[t], buffer), cplex.sum(obs.topLeftCorner.y, cplex.prod(largeNum, slack[2])));
//                cplex.addLe(cplex.negative(cplex.sum(vars.posY[t], -buffer)), cplex.sum(-obs.bottomRightCorner.y, cplex.prod(largeNum, slack[3])));
//                
//                if(obs.isObstacle()){
//                    cplex.addLe(cplex.sum(slack), 3);
//                }else if(obs.isSpeedLimit()){
//                    SpeedLimitRegion2D region = (SpeedLimitRegion2D) obs;
//                    IloIntVar isIn = cplex.intVar(0, 1);
//                    cplex.addLe(cplex.sum(slack), cplex.sum(3, isIn));
//                    cplex.addLe(vars.velX[t], cplex.sum(region.speed, cplex.prod(largeNum, cplex.diff(1, isIn))));
//                    cplex.addGe(vars.velX[t], cplex.sum(-region.speed, cplex.prod(-largeNum, cplex.diff(1, isIn))));
//                    cplex.addLe(vars.velY[t], cplex.sum(region.speed, cplex.prod(largeNum, cplex.diff(1, isIn))));
//                    cplex.addGe(vars.velY[t], cplex.sum(-region.speed, cplex.prod(-largeNum, cplex.diff(1, isIn))));
//                }
//                                
//                
//            }
//        }
//    }
    
    private void generateObstacleConstraints() throws IloException{
        for(ObstacleConstraint cons : scenario.activeSet){
            for(int t = 0; t < scenario.timeSteps; t++){
                cplex.add(cons.getConstraint(vars, t, cplex));
            }
        }
    }
    
    
    private void generateWorldConstraints() throws IloException{
        

        for(int t = 0; t < scenario.timeSteps; t++){
            // World borders
            cplex.add(cplex.not(cplex.le(vars.posX[t], 0)));
            cplex.add(cplex.not(cplex.le(vars.posY[t], 0)));
            cplex.add(cplex.not(cplex.ge(vars.posX[t], scenario.world.getMaxPos().x)));
            cplex.add(cplex.not(cplex.ge(vars.posY[t], scenario.world.getMaxPos().y)));
            
            // Time
            cplex.addEq(vars.time[t], t * scenario.deltaT);
        }
    }
    
    private void generateVehicleConstraints() throws IloException{
        
        // Initial values;
        cplex.addEq(scenario.startPos.x, vars.posX[0]);
        cplex.addEq(scenario.startPos.y, vars.posY[0]);
        if(scenario.startVel == null){
            cplex.addEq(0, vars.velX[0]);
            cplex.addEq(0, vars.velY[0]);
        }else{
            cplex.addEq(scenario.startVel.x, vars.velX[0]);
            cplex.addEq(scenario.startVel.y, vars.velY[0]);   
        }
        
        
        cplex.addEq(0, vars.fin[0]);
        
        
        // Movement
        for(int t = 0; t < scenario.timeSteps - 1; t++){
            cplex.add(vars.verticalThrottle[t]);
            cplex.add(vars.horizontalThrottle[t]);
            cplex.add(vars.velX[t]);
            cplex.add(vars.velY[t]);
            
            // Velocity
            cplex.addEq(vars.velX[t + 1], cplex.sum(vars.velX[t], 
                    cplex.prod(scenario.vehicle.acceleration * scenario.deltaT, vars.horizontalThrottle[t])));
            cplex.addEq(vars.velY[t + 1], cplex.sum(vars.velY[t], 
                    cplex.prod(scenario.vehicle.acceleration * scenario.deltaT, vars.verticalThrottle[t])));
           

            // Position
            cplex.addEq(vars.posX[t + 1], cplex.sum(vars.posX[t], cplex.prod(vars.velX[t], scenario.deltaT)));
            cplex.addEq(vars.posY[t + 1], cplex.sum(vars.posY[t], cplex.prod(vars.velY[t], scenario.deltaT)));
        }
        
        cplex.add(vars.velX[scenario.timeSteps-1]);
        cplex.add(vars.velY[scenario.timeSteps-1]);
        
    }
    
    private IloConstraint iff(IloConstraint a, IloConstraint b) throws IloException{
        IloConstraint r1 = cplex.or(a, cplex.not(b));
        IloConstraint r2 = cplex.or(b, cplex.not(a));
        return cplex.and(r1, r2);
    }
    
    private IloConstraint isTrue(IloNumVar a) throws IloException{
        return cplex.eq(1, a);
    }
    
    private IloConstraint diff(IloNumVar a, IloNumVar b, double val) throws IloException{
        return cplex.le(cplex.abs(cplex.diff(a, b)), val);
    }
    
    private IloConstraint diff(double a, IloNumVar b, double val) throws IloException{
        return cplex.le(cplex.abs(cplex.diff(a, b)), val);
    }
    
    private IloConstraint diff(IloNumVar a, double b, double val) throws IloException{
        return cplex.le(cplex.abs(cplex.diff(a, b)), val);
    }
    public boolean solve(){
        try {
            return cplex.solve();
        } catch (IloException e) {
            e.printStackTrace();
            return false;
        } finally{
            System.out.println("Done");
        }
    }
    
    public Solution getResults() throws UnknownObjectException, IloException{
        double[] valpx = cplex.getValues(vars.posX);
        double[] valpy = cplex.getValues(vars.posY);
        double[] valvx = cplex.getValues(vars.velX);
        double[] valvy = cplex.getValues(vars.velY);
        double[] valhori = cplex.getValues(vars.horizontalThrottle);
        double[] valvert = cplex.getValues(vars.verticalThrottle);
        double[] valfin = cplex.getValues(vars.fin);
        double[] valcfin = cplex.getValues(vars.cfin);
        double[] time = cplex.getValues(vars.time);
        int score = (int) cplex.getObjValue();
        

        
        Solution result = new Solution(scenario.maxTime, scenario.timeSteps);
        for(int t = 0; t < scenario.timeSteps; t++){
            result.pos[t] = new Pos2D(valpx[t], valpy[t]);
            result.vel[t] = new Pos2D(valvx[t], valvy[t]);
            result.fin[t] = (valfin[t] == 1);
            result.cfin[t] = (valcfin[t] == 1);
        }
        result.horiThrottle = valhori;
        result.vertThrottle = valvert;
        result.time = time; 
        result.score = score;
        result.highlightPoints.add(scenario.startPos);
        result.highlightPoints.add(scenario.goal);
        
        for(Region2D cp : vars.checkpoints.keySet()){
            result.checkpoints.put(cp, cplex.getValue(vars.checkpoints.get(cp)));
            Double[] counter = new Double[scenario.timeSteps];
            Double[] change = new Double[scenario.timeSteps];

            for(int t = 0; t < scenario.timeSteps; t++){
                counter[t] = new Double(cplex.getValue(vars.checkpointsCounter.get(cp)[t]));
                change[t] = new Double(cplex.getValue(vars.checkpointsChange.get(cp)[t]));

            }
            result.checkpointCounter.put(cp, counter);
            result.checkpointChange.put(cp, change);

        }
        
        
        return result;
        
    }
    
    public void end(){
        cplex.end();
    }
}
