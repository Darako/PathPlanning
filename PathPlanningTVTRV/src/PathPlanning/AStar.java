package PathPlanning;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class AStar extends SearchMethod {

    private List<Node> open;
	
	public AStar(Map dem, Node init, Node end, short heuristic, boolean withZ, boolean withC) {
		super("AStar", dem, init, end, heuristic, withZ, withC, false);		
		open = new ArrayList<>();		
		for(int j = 0; j < map.get_nrows(); j++) {
	        for(int i = 0; i < map.get_ncols(); i++) {
	            map.get_node(i, j).setParent(null);
	        }
	    }		
		init.setG(0);
		init.setH(get_h(init, init, end));
		init.setF(init.getG()+init.getH());
		open = map.get_succesors_without_obstacles(init);		
	}
	

	@Override
	protected void clear_internal_data() {
		open.clear(); // References in open list lost in DEM clean (new nodes)
		for(int j = 0; j < map.get_nrows(); j++) {
	        for(int i = 0; i < map.get_ncols(); i++) {
	            map.get_node(i, j).setParent(null);
	        }
	    }		
	}

	@Override
	public boolean search() {	
		if(!check_valid_data())
			return false;
		    
		Node nodoActual = start;
		int x=0, y=0;
		float g;
		float h;
		ArrayList<Node> succ = new ArrayList();	  
		ArrayList<Node> path = new ArrayList();	  
		open.add(nodoActual);	
		Collections.sort(open);
		nodoActual = open.remove(0);
		start_cpu_counter();
		while(!nodoActual.equals(goal)) {
			x = nodoActual.getX();
			y = nodoActual.getY();
			succ = map.get_succesors_without_obstacles(nodoActual);
			for(int i = 0; i < succ.size(); i++){
				if(succ.get(i).getParent() == null)
					succ.get(i).setParent(nodoActual);
				g = get_g(nodoActual, succ.get(i), false);
				h = get_h(nodoActual, succ.get(i), goal);
				
				succ.get(i).setF(g,h);
				if(!open.contains(succ.get(i))){
					open.add(succ.get(i));
				}
			}
			Collections.sort(open);
			nodoActual = open.remove(0);	        	
		}
        end_cpu_counter();
		path = get_path(nodoActual); 
		print_open(path);
		return true;
	}
	
	public ArrayList get_path(Node dest) {
		ArrayList<Node> camino = new ArrayList<Node>();
		Node nodoActual = dest;
		while(nodoActual.getParent() != null){
			camino.add(nodoActual);
			nodoActual = nodoActual.getParent();
		}
		camino.add(nodoActual);
		return camino;
    }
	
    /**
     * Print the open list.
     */
    private void print_open(ArrayList<Node> path)
    {   
        System.out.println("Camino :");
        for(int i=0; i < path.size(); i++)
            System.out.println("    " + path.get(i).toString());
        java.io.BufferedReader br = new java.io.BufferedReader(new java.io.InputStreamReader(System.in));
        try{ 
            br.readLine();
        }catch(Exception ioe){}
        System.out.println();
    }	
}
