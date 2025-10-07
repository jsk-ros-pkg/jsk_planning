(define (domain multirobot-time)

    (:requirements :typing :durative-actions)

    (:types robot - robot
            position - position
            trash launch - item)

    (:predicates
        ;; spatial-map
        (can-move ?robot - robot ?from ?to - position)

        ;; robot
        (item-in-place ?item - item ?pos - position)
        (item-in-robot ?item - item ?robot - robot)
        (robot-in-place ?robot - robot ?pos - position)
        (arm-moving ?robot - robot) ;; mutex between move and pick/place
    )

    ;; Move : [?robot] can move from [?from] to [?to].
    (:durative-action move
        :parameters (?robot - robot ?from - position ?to - position)
            :duration (= ?duration 10)
            :condition (and
                        (at start (robot-in-place ?robot ?from))
                        (over all (can-move ?robot ?from ?to))
                        (over all (not (arm-moving ?robot)))
                        )
        :effect (and
                 (at end (not (robot-in-place ?robot ?from)))
                 (at end (robot-in-place ?robot ?to))
                 ))

    ;; Pick : [?robot] can pick [?item] at [?place}.
    (:durative-action pick
        :parameters (?robot - robot ?item - item ?where - position)
            :duration (= ?duration 1)
            :condition (and
                        (over all (robot-in-place ?robot ?where))
                        (at start (item-in-place ?item ?where))
                        )
        :effect (and
                 (at start (not (item-in-place ?item ?where)))
                 (at start (item-in-robot ?item ?robot))
                 (at start (arm-moving ?robot))
                 (at end (not (arm-moving ?robot)))
                 ))


    ;; Place : [?robot] can place [?item] at [?place}.
    (:durative-action place
        :parameters (?robot - robot ?item - item ?where - position)
            :duration (= ?duration 1)
            :condition (and
                        (over all (robot-in-place ?robot ?where))
                        (at start (item-in-robot ?item ?robot)))
        :effect (and
                 (at start (item-in-place ?item ?where))
                 (at start (not (item-in-robot ?item ?robot)))
                 (at start (arm-moving ?robot))
                 (at end (not (arm-moving ?robot)))
                 ))
)
