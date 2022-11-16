(define (domain robot-domain)
    (:requirements :durative-actions :fluents :duration-inequalities)
    (:types food)
    (:predicates
(on_counter ?x - food)
(in_drawer ?x - food)
(on_burner ?x - food)
(gripper_holding ?x - food)
(drawer_open)
(gripper_empty)
(robot_close_to_counter)
	)
    (:action pick_up
        :parameters
            (?x - food)
        :condition
	        (and
		     (gripper_empty)
     (not_in_drawer ?x)
     (robot_close_to_counter)
)
        :effect
	        (and
(gripper_holding ?x)
                	(not (on_counter ?x))
                	(not (on_burder ?x))
                	(not (gripper_empty))
                )
	)

    (:action place_in_drawer
        :parameters
            (?x - food)
        :condition
	        (and
     (gripper_holding ?x)
     (robot_close_to_counter)
     (drawer_open)

)
        :effect
	        (and
                	(not (gripper_holding ?x))
                	(in_drawer ?x)
                	(gripper_empty)
                )
	)
    (:action place_in_counter
        :parameters
            (?x - food)
        :condition
	        (and
     (gripper_holding ?x)
     (robot_close_to_counter)

)
        :effect
	        (and
                	(not (gripper_holding ?x))
                	(on_counter ?x)
                	(gripper_empty)
                )
	)
    (:action open_drawer
        :condition
	        (and
		     (gripper_empty)
     (not drawer_open)
     (robot_close_to_counter)
)
        :effect
	        (and
(drawer_open)
                )
	)
    (:action close_drawer
        :condition
	        (and
		     (gripper_empty)
     (drawer_open)
     (robot_close_to_counter)
)
        :effect
	        (and
                	(not (drawer_open))
                )
	)
    (:action drive_to_counter
        :condition
	        (and
     (not (robot_close_to_counter))
)
        :effect
	        (and
(robot_close_to_counter)
                )
	)
)
