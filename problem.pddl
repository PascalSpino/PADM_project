(define (problem robot1)
    (:domain robot-domain)
    (:objects
        spam sugar
    )
    (:init
        (on_burner sugar)
        (on_counter spam)
 (not robot_close_to_counter)
    )
    (:goal 
(on_counter sugar)
(in_drawer spam)
(not drawer_open)
    )
)
