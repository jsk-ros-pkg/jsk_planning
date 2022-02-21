(define (problem multirobot-time-problem)

    (:domain multirobot-time)

    (:objects
        ;; item
        trash - item

        ;; robot
        spot fetch - robot
        satou-prof - man

        ;; position
        room-73b2-station room-73b2-trash room-trash - position)

    (:init
        ;;
        (robot-in-place fetch room-73b2-station)
        (robot-in-place spot  room-73b2-station)
        ;;
        (item-in-place trash  room-73b2-trash)
        ;; spatial-map
        (can-move fetch room-73b2-station room-73b2-trash)
        (can-move fetch room-73b2-trash room-73b2-station)
        (can-move spot room-73b2-station room-trash)
        (can-move spot room-trash room-73b2-station)
    )
    (:goal
        (and
         (item-in-place trash room-trash)
         (robot-in-place fetch room-73b2-station)
         (robot-in-place spot  room-73b2-station)
        ))

    (:metric minimize (total-time))
)
