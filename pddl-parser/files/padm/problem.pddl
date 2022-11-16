(define (problem robot1)
    (:domain robot-domain)
    (:objects
        spam sugar
    )
    (:init
	(on_burner sugar)
        (on_counter spam)
	(gripper_empty)
 	(robot_far_from_counter)
	(drawer_closed)
    )
    (:goal (and 
	(on_counter sugar)
	(in_drawer spam)
	(drawer_closed)
    ))
)
