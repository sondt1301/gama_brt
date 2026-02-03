model NewModel

import "Traffic.gaml"

global {
	file shp_roads <- file("../includes/roads.shp");
	file shp_nodes <- file("../includes/nodes.shp");
	
	geometry shape <- envelope(shp_roads) + 50; 
	
	graph road_network;
	
	init {
		create road from: shp_roads {
			// Create another road in the opposite direction
			create road {
				num_lanes <- 3;
				shape <- polyline(reverse(myself.shape.points));
				maxspeed <- myself.maxspeed;
				linked_road <- myself;
			}
		}
		
		create intersection from: shp_nodes
				with: [is_traffic_signal::(read("type") = "traffic_signals;crossing")] {
			time_to_change <- 30#s;
		}
        
		// Create a graph representing the road network, with road lengths as weights
		map edge_weights <- road as_map (each::each.shape.perimeter);
		road_network <- as_driving_graph(road, intersection) with_weights edge_weights;
		
		// Initialize the traffic lights
		ask intersection {
			do initialize;
		}
				
		create car number: 1000;
	}
}

species car parent: base_vehicle {
	init {
		vehicle_length <- 1.9 #m;
		max_speed <- 100 #km / #h;
		max_acceleration <- 3.5;
	}
	
	reflex select_next_path when: current_path = nil {
		// A path that forms a cycle

		list<intersection> dst_nodes <- [intersection[0], intersection[1]];
		do compute_path graph: road_network nodes: dst_nodes;
	}
	
	reflex commute when: current_path != nil {
		do drive;
	}
}


experiment NewModel type: gui {
	output synchronized: true {
		display map type: 2d background: #gray {
			species road aspect: base;
			species car aspect: base;
			species intersection aspect: base;
		}
	}
}
