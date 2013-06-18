(define (problem sample)
    (:domain manip)
    (:objects a b c - obj)
    (:init
        (on c a)
        (ontable a)
        (ontable b)
        (clear b)
        (clear c)
        (handempty))
    (:goal
      (and
        (on b c)
        (on a b)
      ))
  )

