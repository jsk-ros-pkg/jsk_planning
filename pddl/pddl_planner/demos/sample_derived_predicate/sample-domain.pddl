(define (domain manip)
    (:requirements :typing :derived-predicates)
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
                 (holding ?obj)))
    (:action putdown
             :parameters (?obj - object)
             :precondition (and
                            (holding ?obj))
      :effect (and
                 (not (holding ?obj))
                 (ontable ?obj)))
    (:action stack
      :parameters (?obj0 ?obj1 - object)
      :precondition (and
                        (holding ?obj0)
                        (clear ?obj1))
      :effect (and
                 (not (holding ?obj0))
                 (on ?obj0 ?obj1)))
    (:action unstack
      :parameters (?obj0 ?obj1 - object)
      :precondition (and
                        (handempty)
                        (on ?obj0 ?obj1)
                        (clear ?obj0))
      :effect (and
                 (not (on ?obj0 ?obj1))
                 (holding ?obj0)))
    (:derived (clear ?obj0 - object)
      (and (not (holding ?obj0))
           (forall (?obj1 - object) (not (on ?obj1 ?obj0)))))
    (:derived (handempty)
      (forall (?obj - object) (not (holding ?obj))))
    )

