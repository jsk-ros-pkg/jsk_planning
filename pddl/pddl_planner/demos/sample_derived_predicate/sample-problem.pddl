(define (problem sample)
    (:domain manip)
    (:objects a b c - object)
    (:init
        (on c a)
        (ontable a)
        (ontable b))
    (:goal
      (and
        (on b c)
        (on a b)
      ))
  )

