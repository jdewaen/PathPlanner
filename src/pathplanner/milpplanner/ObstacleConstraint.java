package pathplanner.milpplanner;

import java.util.List;
import java.util.Map;

import ilog.concert.IloConstraint;
import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.cplex.IloCplex;
import pathplanner.common.Scenario;


public interface ObstacleConstraint {
    
    public IloConstraint getConstraint(SolutionVars vars, int t, Scenario scenario, IloCplex cplex, boolean ignoreSize, List<IloIntVar> slackVars) throws IloException;

}
