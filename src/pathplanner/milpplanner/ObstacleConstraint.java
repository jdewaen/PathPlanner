package pathplanner.milpplanner;

import pathplanner.common.Scenario;
import pathplanner.common.ScenarioSegment;
import ilog.concert.IloConstraint;
import ilog.concert.IloException;
import ilog.cplex.IloCplex;


public interface ObstacleConstraint {
    
    public IloConstraint getConstraint(SolutionVars vars, int t, Scenario scenario, IloCplex cplex) throws IloException;

}
