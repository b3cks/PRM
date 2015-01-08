package test;

import search.State;
import search.heuristics.Heuristic;
public class DistanceHeuristic implements Heuristic{
	private ASVState goalState;
	
	public DistanceHeuristic(ASVState goalState) {
		this.goalState = goalState;
	}
	
	@Override
	public double estimate(State s) {
		// TODO Auto-generated method stub
		ASVState temp = (ASVState) s; 
	    return StateTools.getCost(goalState,temp);
	}

}
