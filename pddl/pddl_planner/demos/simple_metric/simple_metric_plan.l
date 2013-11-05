(ros::roseus "simple_metric_plan")

(load "package://pddl_planner/src/eus-pddl-client.l")
(setq *domain* (instance pddl-domain :init :name 'simple_metric))
(send *domain* :requirements '(:typing :action-costs))
(send *domain* :types '(item))
(send *domain* :predicates
      '(
        (PRED1 ?OBJ - item)
        (PRED2 ?OBJ - item)
        (PRED3 ?OBJ - item)
        ))

(send *domain* :functions '((total-cost)
                            (act1)
                            (act2)
                            (act3)))

(setq *actlist*
      (list
       (instance pddl-action :init
                 :name "act1"
                 :parameters '((?OBJ item))
                 :precondition '((PRED1 ?OBJ))
                 :effect '((NOT (PRED1 ?OBJ))
                           (PRED2 ?OBJ)
                           (increase (total-cost) (act1))
                           ))
       (instance pddl-action :init
                 :name "act2"
                 :parameters '((?OBJ item))
                 :precondition '((PRED2 ?OBJ))
                 :effect '((NOT (PRED2 ?OBJ))
                           (PRED3 ?OBJ)
                           (increase (total-cost) (act2))
                           ))
       (instance pddl-action :init
                 :name "act3"
                 :parameters '((?OBJ item))
                 :precondition '((PRED1 ?OBJ))
                 :effect '((NOT (PRED1 ?OBJ))
                           (PRED3 ?OBJ)
                           (increase (total-cost) (act3))
                           ))
       ))

(dolist (ac *actlist*)
  (send *domain* :add :action ac))

(setq *problem* (instance pddl-problem :init :name 'simple_metric_plan :domain 'simple_metric))

(send *problem* :objects
      '((OBJ1 . item)
        ))

(send *problem* :initial-condition
      '((PRED1 OBJ1)
        (= (total-cost) 0)
        (= (act1) 1)
        (= (act2) 1)
        (= (act3) 3)
        ))

(send *problem* :goal-condition
      '((PRED3 OBJ1)
        ))
(send *problem* :metric '(minimize (total-cost)))

(setq *result* (solve-pddl-planning *domain* *problem* :debug t))
(load "package://pddl_planner/src/pddl-result-graph.l")
(setq *graph* (make-graph-from-pddl-results (list *result*) :node-name :pprint))
(send (make-readable-graph *graph*) :write-to-pdf "test.pdf")
