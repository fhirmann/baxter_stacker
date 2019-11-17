(define (problem baxter_stack) (:domain baxter_domain)
(:objects 
    yellow_block blue_block green_block - block
    loc1 loc2 loc3 - location
)

(:init

    ; yellow_block is on loc1, blue_block is stacked on yellow_block
    ; green_block is on loc2, 

    (hand_empty)
    (on_table yellow_block loc1)
    (on blue_block yellow_block )
    (clear blue_block)
    (on_table green_block loc2)
    (clear green_block)
    (clear loc3)
    ;(clear loc4)

)

(:goal (and

    ; green_block is on loc1, yellow_block is stacked on green_block
    ; blue_block is somewhere, 


    (hand_empty)
    (on_table green_block loc1)
    (on yellow_block green_block)
    (clear yellow_block)
))
)
