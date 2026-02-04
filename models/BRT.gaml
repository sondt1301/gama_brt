model BRT

import "Traffic.gaml"

global {
	file shp_roads <- file("../includes/roads.shp");
	file shp_nodes <- file("../includes/nodes.shp");
	
	geometry shape <- envelope(shp_roads) + 50; 
	
	graph road_network;
	
	float prob_spawn <- 0.5;
	
	init {
		create road from: shp_roads with: [num_lanes::int(read("lanes"))] {
			// Create another road in the opposite direction
			create road {
				num_lanes <- myself.num_lanes;
				shape <- polyline(reverse(myself.shape.points));
				maxspeed <- myself.maxspeed;
				linked_road <- myself;
			}
		}
		
		create intersection from: shp_nodes
				with: [is_traffic_signal::(read("type") = "traffic_signals;crossing")] {
			time_to_change <- 60#s;
		}
        
		// Create a graph representing the road network, with road lengths as weights
		map edge_weights <- road as_map (each::each.shape.perimeter);
		road_network <- as_driving_graph(road, intersection) with_weights edge_weights;
		
		// Initialize the traffic lights
		ask intersection {
			do initialize;
		}
	}
	
	reflex spawn_traffic {
		if flip(prob_spawn) {
			create car;
		} else {
			create motorbike;
		}
	}
	
	reflex spawn_brt {
		if (every(200 #cycle)) {
			create brt_bus;
			write "spawn bus";
		}
	}
}

species car parent: base_vehicle {
	init {
		vehicle_length <- 5.0 #m;
		max_speed <- 50 #km / #h;
		right_side_driving <- true;
		allowed_lanes <- [0, 1, 2];
		num_lanes_occupied <- 2;
	}
	
	reflex select_next_path when: current_path = nil {
		list<intersection> dst_nodes <- [intersection[264], intersection[281]];
		do compute_path graph: road_network nodes: dst_nodes;
	}
	
	reflex commute when: current_path != nil {
		do drive;
	}
}

species motorbike parent: base_vehicle {
	init {
		vehicle_length <- 2.0 #m;
		max_speed <- 40 #km / #h;
		right_side_driving <- true;
		allowed_lanes <- [0, 1, 2, 3];
		num_lanes_occupied <- 1;
	}
	
	reflex select_next_path when: current_path = nil {
		list<intersection> dst_nodes <- [intersection[263], intersection[281]];
		do compute_path graph: road_network nodes: dst_nodes;
	}
	
	reflex commute when: current_path != nil {
		do drive;
	}
}

species brt_bus parent: base_vehicle {
	init {
		vehicle_length <- 18.0 #m;
		max_speed <- 60 #km / #h;
		color <- #green;
		right_side_driving <- false;
		lowest_lane <- 4;
		num_lanes_occupied <- 2;
	}
	
	reflex select_next_path when: current_path = nil {
		list<intersection> dst_nodes <- [intersection[118], intersection[281]];
		do compute_path graph: road_network nodes: dst_nodes;
	}
	
	reflex commute when: current_path != nil {
		do drive;
	}
}

experiment BRT type: gui {
	parameter "Prob spawn car/motorbike" var:prob_spawn min:0.0 max: 1.0;
	
	output synchronized: true {
		display map type: 2d background: #gray {
			species road aspect: base;
			species intersection aspect: base;
			species car aspect: base;
			species motorbike aspect: base;
			species brt_bus aspect: base;
		}
	}
}
