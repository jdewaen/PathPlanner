package pathplanner.milpplanner;

import pathplanner.common.*;
import ilog.cplex.*;
import ilog.cplex.IloCplex.UnknownObjectException;
import ilog.concert.*;

public class CPLEXSolver {
    
    static final double FUZZY_DELTA = 0.01;

    private Scenario2D scenario;
    private IloCplex cplex;
    private SolutionVars vars;
    public CPLEXSolver(Scenario2D scenario){
        this.scenario = scenario;
    }
    
    public void generateConstraints(){
        try {
            cplex = new IloCplex();
            vars = initVars();
            addGoal();
            generateWorldConstraints();
            generateVehicleConstraints();
            
            
        } catch (IloException e) {
            e.printStackTrace();
        }

    }
    
    private SolutionVars initVars() throws IloException{
        SolutionVars result = new SolutionVars();
        
        result.posX = cplex.numVarArray(scenario.timeSteps, 0, Double.MAX_VALUE);
        result.posY = cplex.numVarArray(scenario.timeSteps, 0, Double.MAX_VALUE);
        
        result.velX = cplex.numVarArray(scenario.timeSteps, -Double.MAX_VALUE, Double.MAX_VALUE);
        result.velY = cplex.numVarArray(scenario.timeSteps, -Double.MAX_VALUE, Double.MAX_VALUE);
        
        result.horizontalThrottle = cplex.numVarArray(scenario.timeSteps, -1, 1);
        cplex.add(result.horizontalThrottle);
        result.verticalThrottle = cplex.numVarArray(scenario.timeSteps, -1, 1);
        cplex.add(result.verticalThrottle);

        result.fin = cplex.intVarArray(scenario.timeSteps, 0, 1);
        result.cfin = cplex.intVarArray(scenario.timeSteps, 0, 1);
        cplex.add(result.cfin);

        return result;
    }
    
    private void addGoal() throws IloException{
        cplex.addMaximize(cplex.sum(vars.fin));
        for(int t = 0; t < scenario.timeSteps - 1; t++){
            IloConstraint cfinReq = diff(scenario.goal.x, vars.posX[t], FUZZY_DELTA);
            cfinReq = cplex.and(cfinReq, diff(scenario.goal.y, vars.posY[t], FUZZY_DELTA));
            cfinReq = cplex.and(cfinReq, diff(0, vars.velX[t], FUZZY_DELTA));
            cfinReq = cplex.and(cfinReq, diff(0, vars.velY[t], FUZZY_DELTA));

            cplex.add(iff(cfinReq, isTrue(vars.cfin[t])));
            cplex.add(iff(isTrue(vars.fin[t+1]), cplex.or(isTrue(vars.cfin[t+1]), isTrue(vars.fin[t]))));
        }
        
    }
    
    private void generateWorldConstraints() throws IloException{
        for(Obstacle2D obs : scenario.world.getObstacles()){
            for(int t = 0; t < scenario.timeSteps; t++){

                
                cplex.add(cplex.not(cplex.le(vars.posX[t], 0)));
                cplex.add(cplex.not(cplex.le(vars.posY[t], 0)));
                cplex.add(cplex.not(cplex.ge(vars.posX[t], scenario.world.getMaxPos().x)));
                cplex.add(cplex.not(cplex.ge(vars.posY[t], scenario.world.getMaxPos().y)));

                int largeNum = 99999;
                
                IloIntVar[] slack = cplex.intVarArray(4, 0, 1);;
                cplex.addLe(vars.posX[t], cplex.sum(obs.topLeftCorner.x, cplex.prod(largeNum, slack[0])));
                cplex.addLe(cplex.negative(vars.posX[t]), cplex.sum(-obs.bottomRightCorner.x, cplex.prod(largeNum, slack[1])));
                cplex.addLe(vars.posY[t], cplex.sum(obs.topLeftCorner.y, cplex.prod(largeNum, slack[2])));
                cplex.addLe(cplex.negative(vars.posY[t]), cplex.sum(-obs.bottomRightCorner.y, cplex.prod(largeNum, slack[3])));
                cplex.addLe(cplex.sum(slack), 3);
            }
        }
    }
    
    private void generateVehicleConstraints() throws IloException{
        
        // Initial values;
        cplex.addEq(scenario.startPos.x, vars.posX[0]);
        cplex.addEq(scenario.startPos.y, vars.posY[0]);
        cplex.addEq(0, vars.velX[0]);
        cplex.addEq(0, vars.velY[0]);
        cplex.addEq(0, vars.fin[0]);
        
        
        // Movement
        for(int t = 0; t < scenario.timeSteps - 1; t++){
            cplex.add(vars.verticalThrottle[t]);
            cplex.add(vars.horizontalThrottle[t]);
            cplex.add(vars.velX[t]);
            cplex.add(vars.velY[t]);
            
            // Velocity
            cplex.addEq(vars.velX[t + 1], cplex.sum(vars.velX[t], cplex.prod(scenario.vehicle.acceleration, vars.horizontalThrottle[t])));
            cplex.addEq(vars.velY[t + 1], cplex.sum(vars.velY[t], cplex.prod(scenario.vehicle.acceleration, vars.verticalThrottle[t])));
            
            // Position
            cplex.addEq(0, cplex.sum(vars.posX[t + 1], 
                    cplex.negative(cplex.sum(vars.posX[t], vars.velX[t]))));
            cplex.addEq(0, cplex.sum(vars.posY[t + 1], 
                    cplex.negative(cplex.sum(vars.posY[t], vars.velY[t]))));
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
        
        Solution result = new Solution(scenario.maxTime, scenario.timeSteps);
        for(int t = 0; t < scenario.timeSteps; t++){
            result.pos[t] = new Pos2D(valpx[t], valpy[t]);
            result.vel[t] = new Pos2D(valvx[t], valvy[t]);
            result.fin[t] = (valfin[t] == 1);
            result.cfin[t] = (valcfin[t] == 1);
        }
        result.horiThrottle = valhori;
        result.vertThrottle = valvert;
        
        return result;
        
    }
    
    public void end(){
        cplex.end();
    }
}
