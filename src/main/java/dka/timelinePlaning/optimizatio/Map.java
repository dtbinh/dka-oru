package dka.timelinePlaning.optimizatio;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Random;
import java.util.Set;
import java.util.Vector;





import aima.core.agent.Action;
import aima.core.agent.impl.DynamicAction;

public class Map {
	
	private int[] order ;

	/**
	 * change the whole sequence of cluster 
	 */
	public static Action SEQUENCE_CHANGE = new DynamicAction("Sequence");

	
	
	
//	public Map(int[] order) {				
//		this.order = order;
//	}
	
	
	public Map(Map copyMap) {
		this(copyMap.getState());
	}
	

	public Map(int[] state) {
		this.order = copy(state);		
	}
	
	@Override
	public String toString() {		
		return Arrays.toString(order);
	}
	
	private int[] copy(int[] state) {
		int[] ret = new int[state.length];
		ret = Arrays.copyOf(state ,state.length);
		return ret;
	}



	public int[] getState() {
		return this.order;
	}
	


	public void changeSequence() {
		// this for changing the sequence of a cluster ...
		Random random = new Random();
		int[] rndInx = new int[2];
		rndInx[0] = random.nextInt(this.order.length);
		while(true){
			int tmp = random.nextInt(this.order.length);
			if(rndInx[0] != tmp){
				rndInx[1] = tmp;
				break;
 			}
		}
		int temp = this.order[rndInx[0]]; 
		this.order[rndInx[0]] =  this.order[rndInx[1]];
		this.order[rndInx[1]] = temp;
	    
		//System.out.println(rndInx[0] + " " + rndInx[1]);
		
	}


		
	




	
}
