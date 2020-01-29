(define (domain baxter_domain)

(:requirements :strips :fluents :durative-actions :typing :disjunctive-preconditions)

(:types 
    block - place
    location - place
)

(:predicates 
    (on ?x - block ?y - block)
    (hand_empty)
    (clear ?x - place)
    (on_table ?x - block ?loc - location)
    (holding ?x - block)
)

(:durative-action pick_up
    :parameters (?x - block ?loc - location)
    :duration (= ?duration 2)
    :condition (over all (and
                    (clear ?x) 
                    (on_table ?x ?loc)
                    (hand_empty) 
                    )
                
    )
    :effect (and 
                (at start (and 
                    
                    )
                )
                (at end (and 
                    (holding ?x)
                    (clear ?loc)
                    (not (on_table ?x ?loc))
                    (not (hand_empty))
                    )
                )
    )
)

(:durative-action put_down
    :parameters (?x - block ?loc - location)
    :duration (= ?duration 2)
    :condition (and 
                (over all (and 
                    (clear ?loc)
                    (holding ?x)
                    )
                )
    )
    :effect (and 
                (at start (and 
                    
                    )
                ) 
                (at end (and 
                    (not (clear ?loc))
                    (not (holding ?x))
                    (clear ?x)
                    (hand_empty)
                    (on_table ?x ?loc)
                    )
                )
    )
    
)

(:durative-action stack
    :parameters (?x - block ?y - block)
    :duration (= ?duration 2)
    :condition (and 
                    (over all (and 
                        (clear ?y)
                        (holding ?x)
                        )
                    )
    )
    :effect (and 
                (at start (and 
                    
                    )
                )
                (at end (and 
                    (hand_empty)
                    (on ?x ?y)
                    (clear ?x)
                    (not (clear ?y))
                    (not (holding ?x))
                    )
                )
    )
)

(:durative-action unstack
    :parameters (?x - block ?y - block)
    :duration (= ?duration 2)
    :condition (over all (and 
            (hand_empty)
            (on ?x ?y)
            (clear ?x))
    )
    :effect (and 
                (at start (and 
                    
                    )
                ) 
                
                (at end (and 
                    (not (hand_empty))
                    (holding ?x)
                    (not (clear ?x))
                    (clear ?y)
                    (not (on ?x ?y))
                    )
                )
    )
)

)