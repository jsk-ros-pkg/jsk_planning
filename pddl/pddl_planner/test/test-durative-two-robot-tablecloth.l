;; test-simple-failure-torelant.l
;; Author : Hitoshi Kamada  <h-kamada@jsk.imi.i.u-tokyo.ac.jp>

(require :unittest "lib/llib/unittest.l")
(load "package://pddl_planner/demos/2016_kamada_durative/example.l")

(init-unit-test)

(cond
 ((/= (unix::system "rosrun lpg_planner lpg-1.2") 256)
  (deftest test-2016-kamada-durative ()
    nil))
 (t
  (deftest test-2016-kamada-durative ()
    (setq *result* (solve-pddl-planning *domain* *problem*))
    (assert (not (equal (send (cdr (assoc :state *result*)) :generate-single-relationship) 'error))
            "LPG planner error")
    (setq last-state (last (assoc :step-state *result*)))
    (assert (or
             (equal last-state '(((at-robot robot1 table))))
             (equal last-state '(((at-robot robot2 table))))
             (equal last-state '(((at-robot robot1 table) (at-robot robot2 table))))
             (equal last-state '(((at-robot robot2 table) (at-robot robot1 table)))))
            "goal state")
    )))

(run-all-tests)
(exit)
