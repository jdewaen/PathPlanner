package pathplanner.milpplanner;

import ilog.concert.IloConstraint;
import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.cplex.IloCplex;

import java.util.List;

import pathplanner.common.Scenario;


public interface ObstacleConstraint {
    
    public IloConstraint getConstraint(SolutionVars vars, int t, Scenario scenario, IloCplex cplex, CPLEXSolverConfig config, List<IloIntVar> slackVars) throws IloException;

}
