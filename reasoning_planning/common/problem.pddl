(define (problem task)
(:domain baxter_domain)
(:objects
    blue_block yellow_block green_block - block
    loc1 loc2 loc3 - location
)
(:init


    (clear green_block)
    (clear loc2)
    (clear blue_block)

    (on_table blue_block loc3)
    (on_table green_block loc1)

    (holding yellow_block)

)
(:goal (and
    (hand_empty)
    (on_table green_block loc1)
    (on yellow_block green_block)
    (clear yellow_block)
))
)
