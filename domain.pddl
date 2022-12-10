(define (domain robot-domain)
    (:requirements :strips :negative-preconditions)   
    (:predicates
	(on_counter ?x)
	(in_drawer ?x)
	(on_burner ?x)
	(gripper_holding ?x)
	(drawer_open)
	(drawer_closed)
	(gripper_empty)
	(robot_far_from_counter)
	(robot_close_to_counter)
    )
    
    (:action open_drawer
        :precondition (and
	    (gripper_empty)
     	    (drawer_closed)
     	    (robot_close_to_counter)
	)
        :effect (and 
	    (drawer_open)
	    (not (drawer_closed))
	)
    )

    (:action close_drawer
        :precondition (and
	    (gripper_empty)
     	    (drawer_open)
     	    (robot_close_to_counter)
	)
        :effect (and
            (drawer_closed)
	    (not (drawer_open))
	)
    )
    
    (:action pick_up
        :parameters (?x)
        :precondition (and
	    (gripper_empty)
     	    (robot_close_to_counter)
	)
        :effect (and
	    (gripper_holding ?x)
            (not (on_counter ?x))
            (not (on_burner ?x))
            (not (gripper_empty))
        )
    )

    (:action place_in_drawer
        :parameters (?x)
        :precondition (and
     	    (gripper_holding ?x)
            (robot_close_to_counter)
     	    (drawer_open)
        )
        :effect (and
            (not (gripper_holding ?x))
            (in_drawer ?x)
            (gripper_empty)
        )
    )

    (:action place_on_counter
        :parameters (?x)
        :precondition (and
     	    (gripper_holding ?x)
     	    (robot_close_to_counter)
	)
        :effect (and
            (not (gripper_holding ?x))
            (on_counter ?x)
            (gripper_empty)
        )
    )

    (:action drive_to_counter
        :precondition
     	    (robot_far_from_counter)
        :effect (and
	    (robot_close_to_counter)
	    (not (robot_far_from_counter))
	)
    )
)
