;; test-simple-failure-torelant.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :unittest "lib/llib/unittest.l")
(load "package://pddl_planner/demos/simple_failure_torelant/solve-simple-failure-torelant.l")
(load "package://pddl_planner/test/dummy-action.l")
(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")

(init-unit-test)

(deftest test-simple-failure-torelant ()
  (setq *graph*
        (pddl-plan-to-graph nil :domain *domain* :problem *problem* :failed-nodes *failed-nodes* :debug t))
  (setq step-state (cdr (assoc :step-state *result*)))

  (assert (equal (car step-state) '((at start)))
          "initial state")
  (assert (equal (car (last step-state))
                 '((at elevator) (found elevator-button) (light elevator-button)))
          "goal state"))

(run-all-tests)
(exit)
