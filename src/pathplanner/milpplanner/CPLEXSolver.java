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
