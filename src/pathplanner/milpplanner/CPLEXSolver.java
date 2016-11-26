package pathplanner.milpplanner;

import ilog.concert.IloConstraint;
import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;
import ilog.cplex.IloCplex.UnknownObjectException;

import java.util.HashMap;

import pathplanner.common.Pos2D;
import pathplanner.common.Region2D;
import pathplanner.common.Scenario;
import pathplanner.common.ScenarioSegment;
import pathplanner.common.Solution;

public class CPLEXSolver {

    static final double FUZZY_DELTA = 0.01;
    static final double FUZZY_DELTA_POS = 1;
    static final double MIPGap = 0.1;
    static final int MIN_SPEED_POINTS = 3;
    static final int MAX_SPEED_POINTS = 5;

    private Scenario scen;
    private ScenarioSegment segment;
    private IloCplex cplex;
    private SolutionVars vars;
    public CPLEXSolver(Scenario scenario, ScenarioSegment currentSegment){
        this.scen = scenario;
        this.segment = currentSegment;
    }

    public void generateConstraints(){
        try {
            cplex = new IloCplex();
            cplex.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, MIPGap);
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

        result.posX = cplex.numVarArray(segment.timeSteps, 0, Double.MAX_VALUE);
        result.posY = cplex.numVarArray(segment.timeSteps, 0, Double.MAX_VALUE);
        cplex.add(result.posX);
        cplex.add(result.posY);

        result.velX = cplex.numVarArray(segment.timeSteps, -Double.MAX_VALUE, Double.MAX_VALUE);
        result.velY = cplex.numVarArray(segment.timeSteps, -Double.MAX_VALUE, Double.MAX_VALUE);
        cplex.add(result.velX);
        cplex.add(result.velY);

        result.absVelX = cplex.numVarArray(segment.timeSteps, 0, Double.MAX_VALUE);
        result.absVelY = cplex.numVarArray(segment.timeSteps, 0, Double.MAX_VALUE);
        cplex.add(result.absVelX);
        cplex.add(result.absVelY);


        result.horizontalThrottle = cplex.numVarArray(segment.timeSteps, -1, 1);
        result.verticalThrottle = cplex.numVarArray(segment.timeSteps, -1, 1);
        cplex.add(result.horizontalThrottle);
        cplex.add(result.verticalThrottle);

        result.fin = cplex.intVarArray(segment.timeSteps, 0, 1);
        result.cfin = cplex.intVarArray(segment.timeSteps, 0, 1);
        cplex.add(result.fin);   
        cplex.add(result.cfin);   

        result.time = cplex.numVarArray(segment.timeSteps, 0, Double.MAX_VALUE);
        cplex.add(result.time);

        result.checkpoints = new HashMap<Region2D, IloNumVar>();
        result.checkpointsCounter = new HashMap<Region2D, IloIntVar[]>();
        result.checkpointsChange = new HashMap<Region2D, IloIntVar[]>();

        return result;
    }

    private void addGoal() throws IloException{
        cplex.addMinimize(cplex.diff(segment.timeSteps, cplex.sum(vars.fin)));
        for(int t = 0; t < segment.timeSteps; t++){
            IloConstraint cfinReq = diff(segment.goal.x, vars.posX[t], FUZZY_DELTA_POS);
            cfinReq = cplex.and(cfinReq, diff(segment.goal.y, vars.posY[t], FUZZY_DELTA_POS));

            if(segment.goalVel != null){
                cfinReq = cplex.and(cfinReq, diff(segment.goalVel.x, vars.velX[t], FUZZY_DELTA));
                cfinReq = cplex.and(cfinReq, diff(segment.goalVel.y, vars.velY[t], FUZZY_DELTA));
            }

            cplex.add(iff(cfinReq, isTrue(vars.cfin[t])));
            if(t != segment.timeSteps - 1){
                cplex.add(iff(isTrue(vars.fin[t+1]), cplex.or(isTrue(vars.cfin[t+1]), isTrue(vars.fin[t]))));
            }else{
                cplex.addEq(1, vars.fin[t]);

            }
        }

    }

    private void generateObstacleConstraints() throws IloException{
        for(ObstacleConstraint cons : segment.activeSet){
            for(int t = 0; t < segment.timeSteps; t++){
                cplex.add(cons.getConstraint(vars, t, scen, cplex));
            }
        }
    }


    private void generateWorldConstraints() throws IloException{


        for(int t = 0; t < segment.timeSteps; t++){
            // World borders
            cplex.add(cplex.not(cplex.le(vars.posX[t], scen.vehicle.size)));
            cplex.add(cplex.not(cplex.le(vars.posY[t], scen.vehicle.size)));
            cplex.add(cplex.not(cplex.ge(vars.posX[t], scen.world.getMaxPos().x - scen.vehicle.size)));
            cplex.add(cplex.not(cplex.ge(vars.posY[t], scen.world.getMaxPos().y - scen.vehicle.size)));

            // Time
            cplex.addEq(vars.time[t], t * segment.deltaT);
        }
    }

    private void generateVehicleConstraints() throws IloException{

        // Initial values;
        cplex.addEq(segment.startPos.x, vars.posX[0]);
        cplex.addEq(segment.startPos.y, vars.posY[0]);
        if(segment.startVel == null){
            cplex.addEq(0, vars.velX[0]);
            cplex.addEq(0, vars.velY[0]);
        }else{
            cplex.addEq(segment.startVel.x, vars.velX[0]);
            cplex.addEq(segment.startVel.y, vars.velY[0]);   
        }


        cplex.addEq(0, vars.fin[0]);


        // Movement
        for(int t = 0; t < segment.timeSteps - 1; t++){

            // Velocity
            cplex.addEq(vars.velX[t + 1], cplex.sum(vars.velX[t], 
                    cplex.prod(scen.vehicle.acceleration * segment.deltaT, vars.horizontalThrottle[t])));
            cplex.addEq(vars.velY[t + 1], cplex.sum(vars.velY[t], 
                    cplex.prod(scen.vehicle.acceleration * segment.deltaT, vars.verticalThrottle[t])));

            // Position
            cplex.addEq(vars.posX[t + 1], cplex.sum(vars.posX[t], cplex.prod(vars.velX[t], segment.deltaT)));
            cplex.addEq(vars.posY[t + 1], cplex.sum(vars.posY[t], cplex.prod(vars.velY[t], segment.deltaT)));
        }

        double minSpeed = scen.vehicle.minSpeed;
        double maxSpeed = scen.vehicle.maxSpeed;



        if(!Double.isNaN(minSpeed)){
            double angle = (Math.PI / 2) / (MIN_SPEED_POINTS - 1);
            int largeNum = 999999;

            for(int t = 0; t < segment.timeSteps - 1; t++){
                cplex.addEq(vars.absVelX[t], cplex.abs(vars.velX[t]));
                cplex.addEq(vars.absVelY[t], cplex.abs(vars.velY[t]));


                IloIntVar[] slack = cplex.intVarArray(MIN_SPEED_POINTS - 1, 0, 1);

                double x1 = minSpeed;
                double y1 = 0;
                IloConstraint cons = null;
                for(int i = 1; i < MIN_SPEED_POINTS; i++){
                    double x2 = minSpeed * Math.cos(angle * i);
                    double y2 = minSpeed * Math.sin(angle * i);

                    double a = (y2 - y1) / (x2 - x1);
                    double b = y2 - a * x2;

                    IloConstraint curCons = cplex.ge(vars.absVelY[t], cplex.sum(cplex.sum(
                            cplex.prod(vars.absVelX[t], a), b), cplex.prod(-largeNum, slack[i-1])));

                    if(i == 1){
                        cons = curCons;
                    }else{
                        cons = cplex.and(cons, curCons);
                    }

                    x1 = x2;
                    y1 = y2;
                }

                cplex.add(cons);
                cplex.addLe(cplex.sum(slack), MIN_SPEED_POINTS - 2);

            }
        }

        if(!Double.isNaN(maxSpeed)){
            double angle = (Math.PI / 2) / (MAX_SPEED_POINTS - 1);
            for(int t = 0; t < segment.timeSteps - 1; t++){
                double x1 = maxSpeed;
                double y1 = 0;
                for(int i = 1; i < MAX_SPEED_POINTS; i++){
                    double x2 = maxSpeed * Math.cos(angle * i);
                    double y2 = maxSpeed * Math.sin(angle * i);

                    double a = (y2 - y1) / (x2 - x1);
                    double b = y2 - a * x2;

                    cplex.addLe(vars.absVelY[t], cplex.sum(cplex.prod(vars.absVelX[t], a), b));
                    x1 = x2;
                    y1 = y2;
                }
            }
        }


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



        Solution result = new Solution(segment.maxTime, segment.timeSteps);
        for(int t = 0; t < segment.timeSteps; t++){
            result.pos[t] = new Pos2D(valpx[t], valpy[t]);
            result.vel[t] = new Pos2D(valvx[t], valvy[t]);
            result.fin[t] = (valfin[t] == 1);
            result.cfin[t] = (valcfin[t] == 1);
        }
        result.horiThrottle = valhori;
        result.vertThrottle = valvert;
        result.time = time; 
        result.score = score;
        result.highlightPoints.add(segment.startPos);
        result.highlightPoints.add(segment.goal);

        for(Region2D cp : vars.checkpoints.keySet()){
            result.checkpoints.put(cp, cplex.getValue(vars.checkpoints.get(cp)));
            Double[] counter = new Double[segment.timeSteps];
            Double[] change = new Double[segment.timeSteps];

            for(int t = 0; t < segment.timeSteps; t++){
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
