(define (domain manip)
    (:requirements :typing)
    (:types object)
    (:constants)
    (:predicates
      (on ?obj0 ?obj1 - object)
      (clear ?obj - object)
      (ontable ?obj - object)
      (holding ?obj - object)
      (handempty)
      )
    (:action pickup
      :parameters (?obj - object)
      :precondition (and
                        (ontable ?obj)
                        (clear ?obj)
                        (handempty))
      :effect (and
                 (not (ontable ?obj))
                 (not (clear ?obj))
                 (not (handempty))
                 (holding ?obj)))
    (:action putdown
             :parameters (?obj - object)
             :precondition (and
                            (holding ?obj))
      :effect (and
                 (not (holding ?obj))
                 (ontable ?obj)
                 (clear ?obj)
                 (handempty)))
    (:action stack
      :parameters (?obj0 ?obj1 - object)
      :precondition (and
                        (holding ?obj0)
                        (clear ?obj1))
      :effect (and
                 (not (holding ?obj0))
                 (not (clear ?obj1))
                 (handempty)
                 (on ?obj0 ?obj1)
                 (clear ?obj0)))
    (:action unstack
      :parameters (?obj0 ?obj1 - object)
      :precondition (and
                        (handempty)
                        (on ?obj0 ?obj1)
                        (clear ?obj0))
      :effect (and
                 (not (handempty))
                 (not (on ?obj0 ?obj1))
                 (not (clear ?obj0))
                 (holding ?obj0)
                 (clear ?obj1)))
    )

