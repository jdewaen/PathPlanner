package pathplanner.milpplanner;

import ilog.concert.IloConstraint;
import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;
import ilog.cplex.IloCplex.UnknownObjectException;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.stream.Collectors;

import javax.crypto.spec.IvParameterSpec;

import pathplanner.common.Pos2D;
import pathplanner.common.Scenario;
import pathplanner.common.ScenarioSegment;
import pathplanner.common.Solution;

public class CPLEXSolver {

    static final double FUZZY_DELTA = 0.01;
    static final double MIPGap = 0.1;
    static final double TimeLimit = 120;
    static final int MIN_SPEED_POINTS = 3;
    static final int MAX_SPEED_POINTS = 5;

    private Scenario scen;
    private ScenarioSegment segment;
    private ScenarioSegment nextSegment;
    public  IloCplex cplex;
    private SolutionVars vars;
    private Helper helper;
    public CPLEXSolver(Scenario scenario, ScenarioSegment currentSegment, ScenarioSegment nextSegment){
        this.scen = scenario;
        this.segment = currentSegment;
        this.nextSegment = nextSegment;
    }

    public void generateConstraints(){
        try {
            System.out.println("");
            System.out.println("Init CPLEX");
            cplex = new IloCplex();
            helper = new Helper(cplex);
            cplex.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, MIPGap);
            cplex.setParam(IloCplex.Param.TimeLimit, TimeLimit);
            //            cplex.setParam(IloCplex.Param.MIP.Tolerances.AbsMIPGap, 0.5/scenario.deltaT);
            vars = initVars();
            addGoal();
            generateWorldConstraints();
            generateObstacleConstraints();
            generateVehicleConstraints();  
            System.out.println("Init CPLEX Completed");
            System.out.println("");

        } catch (IloException e) {
            e.printStackTrace();
        }

    }

    private SolutionVars initVars() throws IloException{
        SolutionVars result = new SolutionVars();

        result.posX = cplex.numVarArray(segment.timeSteps, -Double.MAX_VALUE, Double.MAX_VALUE);
        result.posY = cplex.numVarArray(segment.timeSteps, -Double.MAX_VALUE, Double.MAX_VALUE);
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
        
        result.slackVars = new HashMap<PolygonConstraint, Map<Integer,List<IloIntVar>>>();

        return result;
    }

    private void addGoal() throws IloException{
        cplex.addMinimize(cplex.diff(segment.timeSteps, cplex.sum(vars.fin)));
        for(int t = 0; t < segment.timeSteps; t++){
        	
        	
            IloConstraint cfinReq = helper.diff(segment.goal.x, vars.posX[t], segment.positionTolerance);
            cfinReq = cplex.and(cfinReq, helper.diff(segment.goal.y, vars.posY[t], segment.positionTolerance));
//            cfinReq = cplex.and(cfinReq, Line.fromFinish(segment.goal, segment.path.end, 4).getConstraint(vars, t, scen, cplex, true, null));
//            IloConstraint cfinReq = Line.fromFinish(segment.goal, segment.path.end, 4).getConstraint(vars, t, scen, cplex);


            if(segment.goalVel != null){
                cfinReq = cplex.and(cfinReq, helper.diff(segment.goalVel.x, vars.velX[t], FUZZY_DELTA));
                cfinReq = cplex.and(cfinReq, helper.diff(segment.goalVel.y, vars.velY[t], FUZZY_DELTA));
            }
            
//            if(!Double.isNaN(segment.maxGoalVel)){
//                if(t == 0) System.out.println("Adding maximum goal velocity: " + String.valueOf(segment.maxGoalVel));
//                    double angle = (Math.PI / 2) / (MAX_SPEED_POINTS - 1);
//                        double x1 = segment.maxGoalVel + FUZZY_DELTA;
//                        double y1 = 0;
//                        for(int i = 1; i < MAX_SPEED_POINTS; i++){
//                            double x2 = (segment.maxGoalVel + FUZZY_DELTA) * Math.cos(angle * i);
//                            double y2 = (segment.maxGoalVel + FUZZY_DELTA) * Math.sin(angle * i);
//
//                            double a = (y2 - y1) / (x2 - x1);
//                            double b = y2 - a * x2;
//
//                            cfinReq = cplex.and(cfinReq, cplex.le(vars.absVelY[t], cplex.sum(cplex.prod(vars.absVelX[t], a), b)));
//                            x1 = x2;
//                            y1 = y2;
//                        }
//            }

            cplex.add(helper.iff(cfinReq, helper.isTrue(vars.cfin[t])));
            if(t != segment.timeSteps - 1){
                cplex.add(helper.iff(helper.isTrue(vars.fin[t+1]), cplex.or(helper.isTrue(vars.cfin[t+1]), helper.isTrue(vars.fin[t]))));
            }else{
                cplex.addEq(1, vars.fin[t]);

            }
        }

    }
    
    private Map<Integer, List<IloIntVar>> initSlackMap(ObstacleConstraint cons){
        Map<Integer, List<IloIntVar>> slackMap = null;
        if(cons instanceof PolygonConstraint){
            if(!vars.slackVars.containsKey(cons)){
                slackMap = new HashMap<Integer, List<IloIntVar>>();
                vars.slackVars.put((PolygonConstraint) cons, slackMap);
            }else{
                slackMap = vars.slackVars.get(cons);
            }
        }
        return slackMap;
    }
    
    private List<IloIntVar> initSlackVars(Map<Integer, List<IloIntVar>> slackMap, int t){
        List<IloIntVar> slackVars = null;
        if(slackMap != null){
            slackVars = new ArrayList<IloIntVar>();
            slackMap.put(t, slackVars);
        }
        return slackVars;
    }
    
    private void generateObstacleConstraints() throws IloException{
        generateObstacleConstraints(segment, false);
        if( nextSegment != null ) generateObstacleConstraints(nextSegment, true);
    }

    private void generateObstacleConstraints(ScenarioSegment segment, boolean afterFinish) throws IloException{
        
        for(ObstacleConstraint cons : segment.activeSet){
            Map<Integer, List<IloIntVar>> slackMap = initSlackMap(cons);
            List<IloIntVar> lastSlackVars = null;
            for(int t = 0; t < segment.timeSteps; t++){
                List<IloIntVar> slackVars = initSlackVars(slackMap, t);
                
                IloConstraint current = cons.getConstraint(vars, t, scen, cplex, false, slackVars);
                
                if(afterFinish){
                    current = cplex.or(
                                cplex.not(helper.isTrue(vars.fin[t])),
                                current
                            );
                }
                cplex.add(current); 

//                IloConstraint skipCons = cons.preventSkipping(cplex, lastSlackVars, slackVars);
//                if(skipCons != null) cplex.add(skipCons);
//                
//                IloConstraint cornerCons = cons.preventCornerCutting(cplex, lastSlackVars, slackVars);
//                if(cornerCons != null) cplex.add(cornerCons);
                //TODO: fix skipping
                
                lastSlackVars = slackVars;

            }
        }
    }


    private void generateWorldConstraints() throws IloException{


        for(int t = 0; t < segment.timeSteps; t++){
            // World borders
            cplex.add(cplex.not(cplex.le(vars.posX[t], scen.world.getMinPos().x + scen.vehicle.size)));
            cplex.add(cplex.not(cplex.le(vars.posY[t], scen.world.getMinPos().y + scen.vehicle.size)));
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
        System.out.println("Starting at: " + String.valueOf(segment.startPos.x) + " " + String.valueOf(segment.startPos.y));
        if(segment.startVel == null){
            cplex.addEq(0, vars.velX[0]);
            cplex.addEq(0, vars.velY[0]);
            System.out.println("Starting Velocity: 0 0");
        }else{
            cplex.addEq(segment.startVel.x, vars.velX[0]);
            cplex.addEq(segment.startVel.y, vars.velY[0]);
            System.out.println("Starting Velocity: " + String.valueOf(segment.startVel.x) + " " + String.valueOf(segment.startVel.y));
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
        double maxSpeed = segment.maxSpeed;



        if(!Double.isNaN(minSpeed)){
            System.out.println("Adding minimum velocity " + String.valueOf(minSpeed));
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
            System.out.println("Adding maximum velocity " + String.valueOf(maxSpeed));
            double angle = (Math.PI / 2) / (MAX_SPEED_POINTS - 1);
            for(int t = 0; t < segment.timeSteps - 1; t++){
                cplex.addEq(vars.absVelX[t], cplex.abs(vars.velX[t]));
                cplex.addEq(vars.absVelY[t], cplex.abs(vars.velY[t]));
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
    
    private int safeGetIntValue(IloIntVar var){
        try {
            return (int) cplex.getValue(var);
        } catch (UnknownObjectException e) {
            return 0;
        } catch (IloException e) {
            return 0;
        }
    }

    public Solution getResults() throws UnknownObjectException, IloException{
        double[] valpx = cplex.getValues(vars.posX);
        double[] valpy = cplex.getValues(vars.posY);
        double[] valvx = cplex.getValues(vars.velX);
        double[] valvy = cplex.getValues(vars.velY);
        double[] valabsvx = cplex.getValues(vars.absVelX);
        double[] valabsvy = cplex.getValues(vars.absVelY);
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
            result.absVel[t] = new Pos2D(valabsvx[t], valabsvy[t]);
            result.fin[t] = (valfin[t] == 1);
            result.cfin[t] = (valcfin[t] == 1);
        }
        result.horiThrottle = valhori;
        result.vertThrottle = valvert;
        result.time = time; 
        result.score = score;
        result.highlightPoints.add(segment.startPos);
        result.highlightPoints.add(segment.goal);
        
        for(Entry<PolygonConstraint, Map<Integer,List<IloIntVar>>> entry : vars.slackVars.entrySet()){
            PolygonConstraint obs = entry.getKey();
            Map<Integer, List<IloIntVar>> slackMap = entry.getValue();
            Map<Integer, List<Boolean>> resultMap = slackMap.entrySet().stream().map(obsEntry -> {
               return new AbstractMap.SimpleEntry<Integer, List<Boolean>>(obsEntry.getKey(), 
                       obsEntry.getValue().stream().map(val -> safeGetIntValue(val) == 1).collect(Collectors.toList()));
                
            }).collect(Collectors.toMap(resEntry -> resEntry.getKey(), resEntry -> resEntry.getValue()));
            result.slackVars.put(obs.region, resultMap);

        }

//        for(Region2D cp : vars.checkpoints.keySet()){
//            result.checkpoints.put(cp, cplex.getValue(vars.checkpoints.get(cp)));
//            Double[] counter = new Double[segment.timeSteps];
//            Double[] change = new Double[segment.timeSteps];
//
//            for(int t = 0; t < segment.timeSteps; t++){
//                counter[t] = new Double(cplex.getValue(vars.checkpointsCounter.get(cp)[t]));
//                change[t] = new Double(cplex.getValue(vars.checkpointsChange.get(cp)[t]));
//
//            }
//            result.checkpointCounter.put(cp, counter);
//            result.checkpointChange.put(cp, change);
//
//        }


        return result;

    }

    public void end(){
        cplex.end();
    }
}
