###############################################################################
# SA2D TCL Commands
###############################################################################

sta::define_cmd_args "sa2d_set_num_workers" {n}
proc sa2d_set_num_workers { args } {
  sta::parse_key_args "sa2d_set_num_workers" args \
    keys {} \
    flags {}
    
  if { [llength $args] != 1 } {
    utl::error SA2D 100 "sa2d_set_num_workers requires one argument."
  }
  
  set num_workers [lindex $args 0]
  sa2d::set_num_workers $num_workers
}

sta::define_cmd_args "sa2d_set_max_temp" {temp}
proc sa2d_set_max_temp { args } {
  sta::parse_key_args "sa2d_set_max_temp" args \
    keys {} \
    flags {}
    
  if { [llength $args] != 1 } {
    utl::error SA2D 101 "sa2d_set_max_temp requires one argument."
  }
  
  set max_temp [lindex $args 0]
  sa2d::set_max_temp $max_temp
}

sta::define_cmd_args "sa2d_set_min_temp" {temp}
proc sa2d_set_min_temp { args } {
  sta::parse_key_args "sa2d_set_min_temp" args \
    keys {} \
    flags {}
    
  if { [llength $args] != 1 } {
    utl::error SA2D 102 "sa2d_set_min_temp requires one argument."
  }
  
  set min_temp [lindex $args 0]
  sa2d::set_min_temp $min_temp
}

sta::define_cmd_args "sa2d_set_cooling_rate" {rate}
proc sa2d_set_cooling_rate { args } {
  sta::parse_key_args "sa2d_set_cooling_rate" args \
    keys {} \
    flags {}
    
  if { [llength $args] != 1 } {
    utl::error SA2D 103 "sa2d_set_cooling_rate requires one argument."
  }
  
  set cooling_rate [lindex $args 0]
  sa2d::set_cooling_rate $cooling_rate
}

sta::define_cmd_args "sa2d_set_max_iter" {n}
proc sa2d_set_max_iter { args } {
  sta::parse_key_args "sa2d_set_max_iter" args \
    keys {} \
    flags {}
    
  if { [llength $args] != 1 } {
    utl::error SA2D 104 "sa2d_set_max_iter requires one argument."
  }
  
  set max_iter [lindex $args 0]
  sa2d::set_max_iter $max_iter
}

sta::define_cmd_args "sa2d_set_move_budget" {n}
proc sa2d_set_move_budget { args } {
  sta::parse_key_args "sa2d_set_move_budget" args \
    keys {} \
    flags {}
    
  if { [llength $args] != 1 } {
    utl::error SA2D 105 "sa2d_set_move_budget requires one argument."
  }
  
  set move_budget [lindex $args 0]
  sa2d::set_move_budget $move_budget
}

sta::define_cmd_args "sa2d_set_moves_per_iter" {n}
proc sa2d_set_moves_per_iter { args } {
  sta::parse_key_args "sa2d_set_moves_per_iter" args \
    keys {} \
    flags {}
    
  if { [llength $args] != 1 } {
    utl::error SA2D 111 "sa2d_set_moves_per_iter requires one argument."
  }
  
  set moves_per_iter [lindex $args 0]
  sa2d::set_moves_per_iter $moves_per_iter
}

sta::define_cmd_args "sa2d_set_seed" {n}
proc sa2d_set_seed { args } {
  sta::parse_key_args "sa2d_set_seed" args \
    keys {} \
    flags {}
    
  if { [llength $args] != 1 } {
    utl::error SA2D 106 "sa2d_set_seed requires one argument."
  }
  
  set seed [lindex $args 0]
  sa2d::set_seed $seed
}

# Max displacement follows DPL pattern with micron to site conversion
sta::define_cmd_args "sa2d_set_max_displacement" {max_disp_x max_disp_y}
proc sa2d_set_max_displacement { args } {
  sta::parse_key_args "sa2d_set_max_displacement" args \
    keys {} \
    flags {}
    
  if { [llength $args] != 2 } {
    utl::error SA2D 107 "sa2d_set_max_displacement requires two arguments."
  }
  
  set max_disp_x [lindex $args 0]
  set max_disp_y [lindex $args 1]
  
  # Convert from microns to sites (following DPL pattern)
  set tech [ord::get_db_tech]
  set dbu [$tech getDbUnitsPerMicron]
  
  # Get site width (assume uniform for now)
  set block [ord::get_db_block]
  set rows [$block getRows]
  if { [llength $rows] == 0 } {
    utl::error SA2D 108 "No rows found in design."
  }
  
  set first_row [lindex $rows 0]
  set site [$first_row getSite]
  set site_width [$site getWidth]
  set site_height [$site getHeight]
  
  # Convert microns to DBU, then to sites
  set max_disp_x_dbu [expr int($max_disp_x * $dbu)]
  set max_disp_y_dbu [expr int($max_disp_y * $dbu)]
  
  set max_disp_x_sites [expr int($max_disp_x_dbu / $site_width)]
  set max_disp_y_sites [expr int($max_disp_y_dbu / $site_height)]
  
  sa2d::set_max_displacement $max_disp_x_sites $max_disp_y_sites
}

sta::define_cmd_args "sa2d_set_gwtw_interval" {interval}
proc sa2d_set_gwtw_interval { args } {
  sta::parse_key_args "sa2d_set_gwtw_interval" args \
    keys {} \
    flags {}
    
  if { [llength $args] != 1 } {
    utl::error SA2D 109 "sa2d_set_gwtw_interval requires one argument."
  }
  
  set interval [lindex $args 0]
  sa2d::set_gwtw_interval $interval
}

sta::define_cmd_args "sa2d_set_elite_ratio" {ratio}
proc sa2d_set_elite_ratio { args } {
  sta::parse_key_args "sa2d_set_elite_ratio" args \
    keys {} \
    flags {}
    
  if { [llength $args] != 1 } {
    utl::error SA2D 110 "sa2d_set_elite_ratio requires one argument."
  }
  
  set ratio [lindex $args 0]
  sa2d::set_elite_ratio $ratio
}

# LSMC (Large Step Markov Chain) parameters
sta::define_cmd_args "sa2d_set_kick_interval" {interval}
proc sa2d_set_kick_interval { args } {
  sta::parse_key_args "sa2d_set_kick_interval" args \
    keys {} \
    flags {}
    
  if { [llength $args] != 1 } {
    utl::error SA2D 112 "sa2d_set_kick_interval requires one argument."
  }
  
  set interval [lindex $args 0]
  sa2d::set_kick_interval $interval
}

sta::define_cmd_args "sa2d_set_kick_threshold" {threshold}
proc sa2d_set_kick_threshold { args } {
  sta::parse_key_args "sa2d_set_kick_threshold" args \
    keys {} \
    flags {}
    
  if { [llength $args] != 1 } {
    utl::error SA2D 113 "sa2d_set_kick_threshold requires one argument."
  }
  
  set threshold [lindex $args 0]
  sa2d::set_kick_threshold $threshold
}

sta::define_cmd_args "sa2d_set_kick_strength" {strength}
proc sa2d_set_kick_strength { args } {
  sta::parse_key_args "sa2d_set_kick_strength" args \
    keys {} \
    flags {}
    
  if { [llength $args] != 1 } {
    utl::error SA2D 114 "sa2d_set_kick_strength requires one argument."
  }
  
  set strength [lindex $args 0]
  sa2d::set_kick_strength $strength
}

sta::define_cmd_args "sa2d_set_kick_temp_multiplier" {multiplier}
proc sa2d_set_kick_temp_multiplier { args } {
  sta::parse_key_args "sa2d_set_kick_temp_multiplier" args \
    keys {} \
    flags {}
    
  if { [llength $args] != 1 } {
    utl::error SA2D 115 "sa2d_set_kick_temp_multiplier requires one argument."
  }
  
  set multiplier [lindex $args 0]
  sa2d::set_kick_temp_multiplier $multiplier
}

sta::define_cmd_args "sa2d_set_enable_kicks" {enable}
proc sa2d_set_enable_kicks { args } {
  sta::parse_key_args "sa2d_set_enable_kicks" args \
    keys {} \
    flags {}
    
  if { [llength $args] != 1 } {
    utl::error SA2D 114 "sa2d_set_enable_kicks requires one argument."
  }
  
  set enable [lindex $args 0]
  sa2d::set_enable_kicks $enable
}

sta::define_cmd_args "sa2d_set_enable_chain_moves" {enable}
proc sa2d_set_enable_chain_moves { args } {
  sta::parse_key_args "sa2d_set_enable_chain_moves" args \
    keys {} \
    flags {}
    
  if { [llength $args] != 1 } {
    utl::error SA2D 115 "sa2d_set_enable_chain_moves requires one argument."
  }
  
  set enable [lindex $args 0]
  sa2d::set_enable_chain_moves $enable
}

sta::define_cmd_args "sa2d_set_chain_move_interval" {interval}
proc sa2d_set_chain_move_interval { args } {
  sta::parse_key_args "sa2d_set_chain_move_interval" args \
    keys {} \
    flags {}
    
  if { [llength $args] != 1 } {
    utl::error SA2D 116 "sa2d_set_chain_move_interval requires one argument."
  }
  
  set interval [lindex $args 0]
  sa2d::set_chain_move_interval $interval
}

sta::define_cmd_args "sa2d_set_chain_moves_per_round" {moves}
proc sa2d_set_chain_moves_per_round { args } {
  sta::parse_key_args "sa2d_set_chain_moves_per_round" args \
    keys {} \
    flags {}
    
  if { [llength $args] != 1 } {
    utl::error SA2D 117 "sa2d_set_chain_moves_per_round requires one argument."
  }
  
  set moves [lindex $args 0]
  sa2d::set_chain_moves_per_round $moves
}

sta::define_cmd_args "sa2d_set_enable_slides" {enable}
proc sa2d_set_enable_slides { args } {
  sta::parse_key_args "sa2d_set_enable_slides" args \
    keys {} \
    flags {}
    
  if { [llength $args] != 1 } {
    utl::error SA2D 118 "sa2d_set_enable_slides requires one argument."
  }
  
  set enable [lindex $args 0]
  sa2d::set_enable_slides $enable
}

# Basic SA2D placement command
# This runs simulated annealing to optimize placement
# Options:
#   -seed <value>: Random seed (default: 42)
#   -max_displacement <value>: Max displacement in microns (default: 50)
#   -max_temp <value>: Maximum temperature (default: 10.0)
#   -cooling_rate <value>: Cooling rate (default: 0.95)
#   -max_iter <value>: Maximum iterations (default: 100)
#   -move_budget <value>: Total move budget (default: 10000)
#   -moves_per_iter <value>: Moves per iteration (default: 100)
#   -num_workers <value>: Number of parallel workers for GWTW (default: 1)
#   -gwtw_interval <value>: Iterations between GWTW synchronization (default: 50)
#   -elite_ratio <value>: Fraction of workers considered elite in GWTW (default: 0.2)
sta::define_cmd_args "sa2d_simple_place" {
  [-seed seed]
  [-max_displacement disp]
  [-max_temp temp]
  [-cooling_rate rate]
  [-max_iter iter]
  [-move_budget budget]
  [-moves_per_iter moves]
  [-num_workers workers]
  [-gwtw_interval interval]
  [-elite_ratio ratio]
}

proc sa2d_simple_place { args } {
  sta::parse_key_args "sa2d_simple_place" args \
    keys {-seed -max_displacement -max_temp -cooling_rate -max_iter \
          -move_budget -moves_per_iter -num_workers -gwtw_interval -elite_ratio} \
    flags {}
    
  if { [llength $args] != 0 } {
    utl::error SA2D 109 "sa2d_simple_place expects no positional arguments."
  }
  
  # Apply parameters if provided
  if { [info exists keys(-seed)] } {
    sa2d::set_seed $keys(-seed)
  }
  
  if { [info exists keys(-max_displacement)] } {
    set disp $keys(-max_displacement)
    # Handle both single value (same for x and y) and two values
    if { [llength $disp] == 1 } {
      sa2d_set_max_displacement $disp $disp
    } elseif { [llength $disp] == 2 } {
      sa2d_set_max_displacement [lindex $disp 0] [lindex $disp 1]
    } else {
      utl::error SA2D 119 "-max_displacement expects 1 or 2 values"
    }
  }
  
  if { [info exists keys(-max_temp)] } {
    sa2d::set_max_temp $keys(-max_temp)
  }
  
  if { [info exists keys(-cooling_rate)] } {
    sa2d::set_cooling_rate $keys(-cooling_rate)
  }
  
  if { [info exists keys(-max_iter)] } {
    sa2d::set_max_iter $keys(-max_iter)
  }
  
  if { [info exists keys(-move_budget)] } {
    sa2d::set_move_budget $keys(-move_budget)
  }
  
  if { [info exists keys(-moves_per_iter)] } {
    sa2d::set_moves_per_iter $keys(-moves_per_iter)
  }
  
  if { [info exists keys(-num_workers)] } {
    sa2d::set_num_workers $keys(-num_workers)
  }
  
  if { [info exists keys(-gwtw_interval)] } {
    sa2d::set_gwtw_interval $keys(-gwtw_interval)
  }
  
  if { [info exists keys(-elite_ratio)] } {
    sa2d::set_elite_ratio $keys(-elite_ratio)
  }
  
  # Run SA
  sa2d::run
}

# Direct SA2D run command (no parameters)
sta::define_cmd_args "sa2d_run" {}
proc sa2d_run { args } {
  sta::parse_key_args "sa2d_run" args \
    keys {} \
    flags {}
    
  if { [llength $args] != 0 } {
    utl::error SA2D 120 "sa2d_run expects no arguments."
  }
  
  sa2d::run
} 