package dka.timelinePlaning.meta;

import java.util.HashMap;
import java.util.Vector;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.ValueOrderingH;
import org.metacsp.framework.Variable;
import org.metacsp.framework.VariableOrderingH;
import org.metacsp.framework.meta.MetaVariable;
import org.metacsp.meta.symbolsAndTime.MCSData;
import org.metacsp.meta.symbolsAndTime.Schedulable;
import org.metacsp.meta.symbolsAndTime.Schedulable.PEAKCOLLECTION;
import org.metacsp.multi.activity.Activity;
import org.metacsp.multi.activity.SymbolicVariableActivity;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeDomain;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeVariable;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;

import com.vividsolutions.jts.geom.Geometry;



public class InfoGainOrderingMetaConstraint extends Schedulable {

	HashMap<Integer, Pose> roomsPositions;
	HashMap<Integer, Integer> expirations;
	
	public InfoGainOrderingMetaConstraint(VariableOrderingH varOH, ValueOrderingH valOH) {
		super(varOH, valOH);
		this.setPeakCollectionStrategy(PEAKCOLLECTION.BINARY);
	}

	@Override
	public boolean isConflicting(Activity[] peak) {
		if (peak.length < 2) return false;
		TrajectoryEnvelope te1 = (TrajectoryEnvelope)peak[0];
		TrajectoryEnvelope te2 = (TrajectoryEnvelope)peak[1];
		SymbolicVariableActivity act1 = te1.getSymbolicVariableActivity();
		SymbolicVariableActivity act2 = te2.getSymbolicVariableActivity();
		
		if(act1.getSymbols()[1].compareTo(act2.getSymbols()[1]) != 0)
			return true;
		
		return false;
	}

	@Override
	public ConstraintNetwork[] getMetaVariables() {

		if (activities != null && !activities.isEmpty()) {
			Vector<ConstraintNetwork> ret = new Vector<ConstraintNetwork>();
			logger.finest("Doing binary peak collection with " + activities.size() + " activities...");
			Activity[] groundVars = activities.toArray(new Activity[activities.size()]);
			for (int i = 0; i < groundVars.length-1; i++) {
				for (int j = i+1; j < groundVars.length; j++) {
					Bounds bi = new Bounds(groundVars[i].getTemporalVariable().getEST(), groundVars[i].getTemporalVariable().getEET());
					Bounds bj = new Bounds(groundVars[j].getTemporalVariable().getEST(), groundVars[j].getTemporalVariable().getEET());
					if (bi.intersectStrict(bj) != null && isConflicting(new Activity[] {groundVars[i], groundVars[j]})) {
						ConstraintNetwork cn = new ConstraintNetwork(null);
						cn.addVariable(groundVars[i].getVariable());
						cn.addVariable(groundVars[j].getVariable());
						ret.add(cn);
					}
				}
			}
			if (!ret.isEmpty()) {
				return ret.toArray(new ConstraintNetwork[ret.size()]);			
			}
		}
		return (new ConstraintNetwork[0]);

	}

	@Override
	public ConstraintNetwork[] getMetaValues(MetaVariable metaVariable) {	
		Vector<ConstraintNetwork> ret = new Vector<ConstraintNetwork>();
		ConstraintNetwork conflict = metaVariable.getConstraintNetwork();
		Variable[] vars = conflict.getVariables();
		TrajectoryEnvelope te0 = (TrajectoryEnvelope)vars[0];
		TrajectoryEnvelope te1 = (TrajectoryEnvelope)vars[1];
		
		//if both are moving
		int first = Integer.valueOf(te0.getSymbolicVariableActivity().getSymbols()[1]); 
		int second = Integer.valueOf(te1.getSymbolicVariableActivity().getSymbols()[1]);
		System.out.println(first + " " + second);
		
		//if first zero, second move
		//first zero, second sense
		//if first sense, second zero
		//if first move, second zero
		
		if(first == 0 && te1.getSymbolicVariableActivity().getSymbols()[0].contains("move")){
			ConstraintNetwork cn0 = new ConstraintNetwork(null);
			Constraint dur0 = getDurationConstraint(te1, first, second);
			Constraint before0 = getBeforeConstraint(te0,te1);
			cn0.addConstraints(dur0, before0);
			ret.add(cn0);			
		}
		else if (first == 0 && te1.getSymbolicVariableActivity().getSymbols()[0].contains("sense")){
			ConstraintNetwork cn0 = new ConstraintNetwork(null);
			Constraint dur0 = getDurationConstraint(findTrajectoryEnvelope("move",second), first, second);
			Constraint before0 = getBeforeConstraint(te0,findTrajectoryEnvelope("move",second));
			cn0.addConstraints(dur0, before0);
			ret.add(cn0);			
		
		}
		else if (te0.getSymbolicVariableActivity().getSymbols()[0].contains("move") && second == 0){
			ConstraintNetwork cn0 = new ConstraintNetwork(null);
			Constraint dur0 = getDurationConstraint(te0, first, second);
			Constraint before0 = getBeforeConstraint(te1,te0);
			cn0.addConstraints(dur0, before0);
			ret.add(cn0);			
		
		}
		else if (te0.getSymbolicVariableActivity().getSymbols()[0].contains("sense") && second == 0){
			ConstraintNetwork cn0 = new ConstraintNetwork(null);
			Constraint dur0 = getDurationConstraint(findTrajectoryEnvelope("move",first), first, second);
			Constraint before0 = getBeforeConstraint(te1,findTrajectoryEnvelope("move",first));
			cn0.addConstraints(dur0, before0);
			ret.add(cn0);			
		
		}// the if above was for handling the start situation
		else if(te0.getSymbolicVariableActivity().getSymbols()[0].contains("move") && te1.getSymbolicVariableActivity().getSymbols()[0].contains("move")){
			
			ConstraintNetwork cn0 = new ConstraintNetwork(null);
			Constraint dur0 = getDurationConstraint(te1, first, second);
			Constraint before0 = getBeforeConstraint(findTrajectoryEnvelope("sense",first),te1);
			Constraint exp0 = getExpirationConstraint(findTrajectoryEnvelope("sense",first),findTrajectoryEnvelope("sense",second), expirations.get(first));
			cn0.addConstraints(dur0, before0, exp0);
			ret.add(cn0);

			ConstraintNetwork cn1 = new ConstraintNetwork(null);
			Constraint dur1 = getDurationConstraint(te0, second, first);
			Constraint before1 = getBeforeConstraint(findTrajectoryEnvelope("sense",second),te0);
			Constraint exp1 = getExpirationConstraint(findTrajectoryEnvelope("sense",second),findTrajectoryEnvelope("sense",first), expirations.get(second));
			cn1.addConstraints(dur1, before1, exp1);
			ret.add(cn1);
			
			return ret.toArray(new ConstraintNetwork[ret.size()]);
		}
		else if(te0.getSymbolicVariableActivity().getSymbols()[0].contains("sense") && te1.getSymbolicVariableActivity().getSymbols()[0].contains("sense")){
			
			ConstraintNetwork cn0 = new ConstraintNetwork(null);
			Constraint dur0 = getDurationConstraint(findTrajectoryEnvelope("move",second), first, second);
			Constraint before0 = getBeforeConstraint(te0, findTrajectoryEnvelope("move",second));
			Constraint exp0 = getExpirationConstraint(te0,te1, expirations.get(first));
			cn0.addConstraints(dur0, before0, exp0);
			ret.add(cn0);

			ConstraintNetwork cn1 = new ConstraintNetwork(null);
			Constraint dur1 = getDurationConstraint(findTrajectoryEnvelope("move",first), second, first);
			Constraint before1 = getBeforeConstraint(te1, findTrajectoryEnvelope("move",first));
			Constraint exp1 = getExpirationConstraint(te1,te0, expirations.get(second));
			cn0.addConstraints(dur1, before1, exp1);
			ret.add(cn1);
			
			return ret.toArray(new ConstraintNetwork[ret.size()]);
		}
		else if(te0.getSymbolicVariableActivity().getSymbols()[0].contains("sense") && te1.getSymbolicVariableActivity().getSymbols()[0].contains("move")){
			
			ConstraintNetwork cn0 = new ConstraintNetwork(null);
			Constraint dur0 = getDurationConstraint(te1, first, second);
			Constraint before0 = getBeforeConstraint(te0,te1);
			Constraint exp0 = getExpirationConstraint(te0,findTrajectoryEnvelope("sense",second), expirations.get(first));
			cn0.addConstraints(dur0, before0, exp0);
			ret.add(cn0);

			ConstraintNetwork cn1 = new ConstraintNetwork(null);
			Constraint dur1 = getDurationConstraint(findTrajectoryEnvelope("move",first), second, first);
			Constraint before1 = getBeforeConstraint(findTrajectoryEnvelope("sense",second),findTrajectoryEnvelope("move",first));
			Constraint exp1 = getExpirationConstraint(findTrajectoryEnvelope("sense",second),te0, expirations.get(second));
			cn1.addConstraints(dur1, before1, exp1);
			ret.add(cn1);
			
			return ret.toArray(new ConstraintNetwork[ret.size()]);
		}
		else if(te0.getSymbolicVariableActivity().getSymbols()[0].contains("move") && te1.getSymbolicVariableActivity().getSymbols()[0].contains("sense")){
			
			ConstraintNetwork cn0 = new ConstraintNetwork(null);
			Constraint dur0 = getDurationConstraint(findTrajectoryEnvelope("move",second), first, second);
			Constraint before0 = getBeforeConstraint(findTrajectoryEnvelope("sense",first),findTrajectoryEnvelope("move",second));
			Constraint exp0 = getExpirationConstraint(findTrajectoryEnvelope("sense",first), te1, expirations.get(first));
			cn0.addConstraints(dur0, before0, exp0);
			ret.add(cn0);

			ConstraintNetwork cn1 = new ConstraintNetwork(null);
			Constraint dur1 = getDurationConstraint(te0, second, first);
			Constraint before1 = getBeforeConstraint(te1,te0);
			Constraint exp1 = getExpirationConstraint(te1, findTrajectoryEnvelope("sense",first), expirations.get(second));
			cn1.addConstraints(dur1, before1, exp1);

			ret.add(cn1);
			
			return ret.toArray(new ConstraintNetwork[ret.size()]);
		}
		return ret.toArray(new ConstraintNetwork[ret.size()]);

	}

	private AllenIntervalConstraint getBeforeConstraint(TrajectoryEnvelope te0, TrajectoryEnvelope te1) {
		AllenIntervalConstraint before = new AllenIntervalConstraint(AllenIntervalConstraint.Type.BeforeOrMeets, new Bounds(this.beforeParameter, APSPSolver.INF));
		//AllenIntervalConstraint before = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Meets);
		before.setFrom(te0);			
		before.setTo(te1);				
		return before;
	}

	private AllenIntervalConstraint getExpirationConstraint(TrajectoryEnvelope te0, TrajectoryEnvelope te1, int maxExp) {
		AllenIntervalConstraint before = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Before, new Bounds(1, maxExp));
		before.setFrom(te0);			
		before.setTo(te1);				
		return before;
	}
	
	private TrajectoryEnvelope findTrajectoryEnvelope(String symbol, int roomId) {
		Variable[] vars = this.getGroundSolver().getVariables();
		for (int i = 0; i < vars.length; i++) {
			TrajectoryEnvelope te = (TrajectoryEnvelope)vars[i];
			if(te.getSymbolicVariableActivity().getSymbols()[0].contains(symbol) && 
					te.getSymbolicVariableActivity().getSymbols()[1].compareTo(String.valueOf(roomId)) == 0){
				return te;
			}
		}
		return null;
	}

	private Constraint getDurationConstraint(TrajectoryEnvelope te0, int first, int second) {
		
		long duration = (long) getDistBetween(roomsPositions.get(first), roomsPositions.get(second));		
		AllenIntervalConstraint ret = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(duration, duration));
		ret.setFrom(te0);			
		ret.setTo(te0);
		
		return ret;
	}

	private static double getDistBetween(Pose pose, Pose pose2) {
		double x1 = pose.getX();
		double x2 = pose2.getX();
		double y1 = pose.getY();
		double y2 = pose2.getY();
		return Math.hypot(x2-x1, y2-y1);
	}
	
	@Override
	public void draw(ConstraintNetwork network) {
		// TODO Auto-generated method stub

	}

	@Override
	public ConstraintSolver getGroundSolver() {
		return (((TrajectoryEnvelopeSolver)this.metaCS.getConstraintSolvers()[0]));	}

	@Override
	public String toString() {		
		return this.getClass().getName();
	}

	@Override
	public String getEdgeLabel() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Object clone() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public boolean isEquivalent(Constraint c) {
		// TODO Auto-generated method stub
		return false;
	}

	public void setRoomPositions(HashMap<Integer, Pose> roomsPositions) {
		this.roomsPositions = roomsPositions;
		
	}
	
	public void setExpirations(HashMap<Integer, Integer> expirations) {
		this.expirations = expirations;
	}

}
