package dka.timelinePlaning.optimizatio;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Random;
import java.util.Vector;
import java.util.logging.Level;

import org.metacsp.framework.Variable;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint.Type;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.time.Bounds;
import org.metacsp.utility.logging.MetaCSPLogging;

import com.vividsolutions.jts.geom.Coordinate;

import aima.core.search.framework.HeuristicFunction;
import dka.timelinePlaning.meta.DKASingleRobotPlanner;


public class TValidityHeuristic implements HeuristicFunction {

	HashMap<Integer, Pose> roomsPositions;
	HashMap<Integer, Bounds> expirations;
	
	public double h(Object state) {
		
		Map m = (Map)state;
		int[] order = m.getState();
		TrajectoryEnvelopeSolver solver = new TrajectoryEnvelopeSolver(0, 100000);
		Variable[] vars = solver.createVariables(order.length*2+1);
		
		
		Vector<TrajectoryEnvelope> tes = new Vector<TrajectoryEnvelope>();
		Vector<TrajectoryEnvelope> sensings = new Vector<TrajectoryEnvelope>();
		Vector<AllenIntervalConstraint> cons = new Vector<AllenIntervalConstraint>();
		Vector<AllenIntervalConstraint> consWOexp = new Vector<AllenIntervalConstraint>();
		
		//initial case
		TrajectoryEnvelope startTE = (TrajectoryEnvelope)vars[0];
		startTE.getSymbolicVariableActivity().setSymbolicDomain("senseAt","0");
		startTE.setFootprint(getTurtlebotFootprint());
		startTE.setTrajectory(new Trajectory(new Pose[]{roomsPositions.get(0)}));
		startTE.setRobotID(1);
		tes.add(startTE);
		
		AllenIntervalConstraint startTEduration = new AllenIntervalConstraint(Type.Duration, new Bounds(10, 10));
		startTEduration.setFrom(startTE);
		startTEduration.setTo(startTE);
		cons.add(startTEduration);
		consWOexp.add(startTEduration);
		
		TrajectoryEnvelope prevSense = startTE;
		int i = 1;
		for (int j = 0; j < order.length; j++) {
			
			TrajectoryEnvelope moveTE = (TrajectoryEnvelope)vars[i];
			moveTE.getSymbolicVariableActivity().setSymbolicDomain("moveTo",Integer.toString(order[j]));			 
			Trajectory traj2 = new Trajectory(new Pose[]{roomsPositions.get(order[j])});
			moveTE.setFootprint(getTurtlebotFootprint());
			moveTE.setTrajectory(traj2);
			moveTE.setRobotID(1);
			tes.add(moveTE);
			i++;
			//prevSense meets moveTo  
			AllenIntervalConstraint prevSenseMeetsMoveTo = new AllenIntervalConstraint(Type.Meets);
			prevSenseMeetsMoveTo.setFrom(prevSense);
			prevSenseMeetsMoveTo.setTo(moveTE);
			cons.add(prevSenseMeetsMoveTo);
			consWOexp.add(prevSenseMeetsMoveTo);
			
			
			TrajectoryEnvelope senseTE = (TrajectoryEnvelope)vars[i];
			senseTE.getSymbolicVariableActivity().setSymbolicDomain("senseAt",Integer.toString(order[j]));
			senseTE.setFootprint(getTurtlebotFootprint());
			senseTE.setTrajectory(traj2);
			senseTE.setRobotID(1);
			tes.add(senseTE);
			sensings.add(senseTE);
			i++;
		
			AllenIntervalConstraint sensingDuration = new AllenIntervalConstraint(Type.Duration, new Bounds(10, 10));
			sensingDuration.setFrom(senseTE);
			sensingDuration.setTo(senseTE);
			cons.add(sensingDuration);
			consWOexp.add(sensingDuration);
			
			AllenIntervalConstraint moveMeetsSense = new AllenIntervalConstraint(Type.Meets);
			moveMeetsSense.setFrom(moveTE);
			moveMeetsSense.setTo(senseTE);
			cons.add(moveMeetsSense);
			consWOexp.add(moveMeetsSense);
			
			
			
			prevSense = senseTE;
		}
		
		//add expiration time		
		for (int j = 0; j < sensings.size() - 1; j++) {
			int roomId = Integer.valueOf(sensings.get(j).getSymbolicVariableActivity().getSymbols()[1]); 
			AllenIntervalConstraint expirationBefore = new AllenIntervalConstraint(Type.Before, new Bounds(1, expirations.get(roomId).max));
			expirationBefore.setFrom(sensings.get(j));
			expirationBefore.setTo(sensings.lastElement());
			cons.add(expirationBefore);			
		}

		int[] score = new int[order.length - 1];
		if(solver.addConstraints(cons.toArray(new AllenIntervalConstraint[cons.size()]))){
			for (int j = 0; j < sensings.size() - 1; j++) {
				int roomId = Integer.valueOf(sensings.get(j).getSymbolicVariableActivity().getSymbols()[1]); 
				int lastingInfo = (int) (sensings.get(j).getTemporalVariable().getEET() + expirations.get(roomId).min); 
				score[j] = (int) (lastingInfo - sensings.lastElement().getTemporalVariable().getEET());
			}
			Arrays.sort(score);
			System.out.println("feasible: " + score[0]);
			//System.out.println("--" + Arrays.toString(score));
			return -score[0];
		}else{
			solver.addConstraints(consWOexp.toArray(new AllenIntervalConstraint[consWOexp.size()]));
			for (int j = 0; j < sensings.size() - 1; j++) {
				int roomId = Integer.valueOf(sensings.get(j).getSymbolicVariableActivity().getSymbols()[1]);
				int lastingInfo = (int) (sensings.get(j).getTemporalVariable().getEET() + expirations.get(roomId).min); 
				score[j] = (int) (lastingInfo - sensings.lastElement().getTemporalVariable().getEET());
			}
			Arrays.sort(score);
			//System.out.println(Arrays.toString(score));
			//System.out.println("infeasible: " + score[0]);
			return -score[0];
		}
		
	}
	

	public void setRoomPositions(HashMap<Integer, Pose> roomsPositions) {
		this.roomsPositions = roomsPositions;
		
	}
	
	public void setExpirations(HashMap<Integer, Bounds> expirations) {
		this.expirations = expirations;
	}
	
	public  Coordinate[] getTurtlebotFootprint(){
		Coordinate[] ret = new Coordinate[6];		
		ret[0] = new Coordinate(1.2, 0.0);
		ret[1] = new Coordinate(0.6, 1.2);
		ret[2] = new Coordinate(-0.6, 1.2);
		ret[3] = new Coordinate(-1.2, 0.0);
		ret[4] = new Coordinate(-0.6, -1.2);
		ret[5] = new Coordinate(0.6, -1.2);
		
		return ret;
	}


	
}