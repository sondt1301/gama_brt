model BRT

import "Traffic.gaml"

global {
	file shp_roads <- file("../includes/roads.shp");
	file shp_nodes <- file("../includes/nodes.shp");
	
	geometry shape <- envelope(shp_roads) + 50; 
	
	graph road_network;
	
	// Params to keep track of throughput
	int monitored_intersection <- 43;
	int intersection_pass_count <- 0;
	// Params to keep track of avg speed
	float brt_speed_sum <- 0.0;
	int brt_speed_samples <- 0;
	
	// Lists for graph
	list<int> throughput_series <- [];
	list<float> brt_travel_times <- [];
	list<float> brt_times_stop1 <- [];
	list<float> brt_times_stop2 <- [];
	list<float> brt_speed_series <- [];
	
	
	float seed <- 42.0;
	bool dynamic_lane_policy <- false;
	int bus_stop_duration <- 5;
	float prob_vehicle_spawn <- 0.5;
	float prob_car_motor_spawn <- 0.3;
	int traffic_signal_time <- 30;
	int brt_cycle <- 121;
	
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
			time_to_change <- traffic_signal_time#s;
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
		if (flip(prob_vehicle_spawn)) {
			if flip(prob_car_motor_spawn) {
				create car;
			} else {
				create motorbike;
			}
		}
	}
	
	reflex spawn_brt {
		if (every(brt_cycle #cycle)) {
			create brt_bus;
			write "spawn bus";
		}
	}
	
	reflex count_intersection_flow {
    	intersection target <- intersection[monitored_intersection];

    	ask (car + motorbike + brt_bus) {
	        if ((location distance_to target.location) < 10) {
	            if (!counted_at_intersection) {
	                intersection_pass_count <- intersection_pass_count + 1;
	                counted_at_intersection <- true;
	            }
	        } else {
	            counted_at_intersection <- false;
	        }
	    }

	    if intersection_pass_count > 0 {
	    	if (every(120 #s)) {
	    		throughput_series <- throughput_series + intersection_pass_count;
				write "Number of vehicle throughput every 2 mins: " + intersection_pass_count;
				intersection_pass_count <- 0;
			}
	    }
	}
	
	reflex report_brt_speed {
	    if (every(120 #s)) {
	        if (brt_speed_samples > 0) {
	            float avg_speed <- (brt_speed_sum / brt_speed_samples) * 3.6;
	            write "Average free-running BRT speed (km/h): " + avg_speed;	
	            brt_speed_series <- brt_speed_series + avg_speed;	            
	        } else {
	            write "No valid BRT speed samples";
	        }
	
	        brt_speed_sum <- 0.0;
	        brt_speed_samples <- 0;
	    }
	}	
}

species car parent: base_vehicle {
	list normal_lanes <- [0, 1, 2];
	
	init {
		vehicle_length <- 5.0 #m;
		max_speed <- rnd(40, 60) #km / #h;
		max_acceleration <- rnd(1.5, 3.5);
		right_side_driving <- true;
		allowed_lanes <- normal_lanes;
		num_lanes_occupied <- 2;
		lane_change_limit <- 2;
		color <- #yellow;
	}
	
	reflex adapt_lane_policy when: dynamic_lane_policy {
        bool jammed <- (real_speed < 10 #km/#h);
        if (jammed) {
            allowed_lanes <- [0,1,2,3,4];
        } else {
            allowed_lanes <- normal_lanes;
        }
    }
	
	reflex select_next_path when: current_path = nil {
		list<intersection> dst_nodes <- [intersection[264], intersection[281]];
		do compute_path graph: road_network nodes: dst_nodes;
	}
	
	reflex commute when: current_path != nil {
		do drive;
		
		if (current_path = nil) {
			do unregister;
			do die;
		}
	}
}

species motorbike parent: base_vehicle {	
	list normal_lanes <- [0, 1, 2, 3];
	
	init {
		vehicle_length <- 2.0 #m;
		max_speed <- rnd(30, 50) #km / #h;
		max_acceleration <- rnd(1.5, 3.5);
		right_side_driving <- true;
		allowed_lanes <- normal_lanes;
		num_lanes_occupied <- 1;
		lane_change_limit <- 2;
		color <- #red;
	}
	
	reflex adapt_lane_policy when: dynamic_lane_policy {
		
        bool jammed <- (real_speed < 10 #km/#h);

        if (jammed) {
            allowed_lanes <- [0,1,2,3,4,5];
        } else {
            allowed_lanes <- normal_lanes;
        }
    }
	
	reflex select_next_path when: current_path = nil {
		list<intersection> dst_nodes <- [intersection[263], intersection[281]];
		do compute_path graph: road_network nodes: dst_nodes;
	}
	
	reflex commute when: current_path != nil {
		do drive;
		
		if (current_path = nil) {
			do unregister;
			do die;
		}
	}
}

species brt_bus parent: base_vehicle {
	int waiting <- 0;              
	list<intersection> stops <- [intersection[124], intersection[256]];
	int stop_index <- 0;      
	float start_time <- 0.0;
	list<float> stop_times <- [];
			
	init {
		road_graph <- road_network;
		vehicle_length <- 18.0 #m;
		max_speed <- 60 #km / #h;
		max_acceleration <- 3.5;
		color <- #green;
		right_side_driving <- false;
		lowest_lane <- 4;
		num_lanes_occupied <- 2;
		lane_change_limit <- 0;
		start_time <- time;
	}
	
	reflex select_next_path when: current_path = nil {
		// Ensure buses go through stops
		list<intersection> dst_nodes <- [intersection[118]] + stops + [intersection[281]];
		do compute_path graph: road_graph nodes: dst_nodes;
	}
	
	reflex commute when: current_path != nil {
		// Check if bus is at the stop
		if (stop_index < length(stops)) {
			intersection target_stop <- stops[stop_index];
			if ((self.location distance_to target_stop.location) < 10) {
				// Record arrival time at the stop
            	stop_times <- stop_times + (time - start_time);
				
				waiting <- bus_stop_duration;
				stop_index <- stop_index + 1;
				write "Bus stopping at stop " + target_stop.index;
				return;
			}
		}
		// If bus is at the stop, stops
		if (waiting > 0) {
			waiting <- waiting - 1;
			return;
		}
		
		// Measure speed
	    brt_speed_sum <- brt_speed_sum + speed;
	    brt_speed_samples <- brt_speed_samples + 1;
		
		do drive;
		
		if (current_path = nil) {
			// Record bus time
			float total_travel_time <- time - start_time;
			float stop1 <- (length(stop_times) > 0 ? stop_times[0] : 0);
		    float stop2 <- (length(stop_times) > 1 ? stop_times[1] : 0);
		
		    ask world {
		        brt_travel_times <- brt_travel_times + (total_travel_time - (stop1+stop2));
		        brt_times_stop1 <- brt_times_stop1 + stop1;
		        brt_times_stop2 <- brt_times_stop2 + stop2;
		    }
			
	        write "Bus total travel time in seconds: " + total_travel_time;
	        write "Segment times in seconds: " + stop_times;
			
			
			do unregister;
			do die;
		}
	}
}

experiment BRT type: gui {
	parameter "Enable other vehicles in the BRT lane" var:dynamic_lane_policy;
	parameter "Waiting time at each bus stop" var:bus_stop_duration min:1 max:10;
	parameter "Prob spawn for both car and motorbike at a step" var:prob_vehicle_spawn min:0.0 max: 1.0;
	parameter "Prob spawn car/motorbike" var:prob_car_motor_spawn min:0.0 max: 1.0;
	parameter "Traffic light change time" var:traffic_signal_time min:0 max:200;
	parameter "Number of cycles the BRT bus spawn" var:brt_cycle min:1;
	
	output synchronized: true {
		display map type: 2d background: #gray {
			species road aspect: base;
			species intersection aspect: base;
			species car aspect: base;
			species motorbike aspect: base;
			species brt_bus aspect: base;
		}
		display "Throughput chart" type: 2d{
		    chart "Throughput over time" type: histogram
		    series_label_position: yaxis
    		x_label: "Time (2-min steps)"
		    {
		        data "Vehicles / 2 min" value: throughput_series color: #purple;
		    }
		}
		
		display "BRT travel time" type: 2d {
		    chart "BRT travel time (seconds)" type: histogram
		    style: stack
		    x_label: "Bus index"
		    {
		        data "Time at stop 1"
		            value: brt_times_stop1
		            color: #blue;
		
		        data "Time at stop 2"
		            value: brt_times_stop2
		            color: #green;
		            
		        data "Remaining time to endpoint"
		            value: brt_travel_times
		            color: #red;
		    }
		}
		display "BRT speed chart" type: 2d {
		    chart "Average BRT speed (km/h)" type: series
		    x_label: "Time (2-min steps)"
		    y_label: "Speed (km/h)"
		    {
		        data "BRT speed"
		            value: brt_speed_series
		            color: #brown
		            style: line
		            marker: true;
		
		    }
		}
	}
}
