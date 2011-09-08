(load "package://pddl_planner/src/eus-pddl-client.l")

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'pr2_action))
(send *domain* :requirements '(:typing))
(send *domain* :types '(object))
(send *domain* :predicates
      '((ON ?OBJ1 ?OBJ2 - object)
        (GRASP ?OBJ - object)))

;;
;; action
;;
(setq *actlist*
      (list
       (instance pddl-action :init
                 :name "pick"
                 :parameters '((?OBJ object))
                 :precondition '((forall (?obj - object)
                                         (NOT (GRASP ?obj))))
                 :effect '((GRASP ?OBJ)
			   (forall (?base - object)
				   (NOT (ON ?OBJ ?base)))))
       (instance pddl-action :init
                 :name "put"
                 :parameters '((?OBJ ?BASE object))
                 :precondition '((GRASP ?OBJ))
                 :effect '((NOT (GRASP ?OBJ))
			   (ON ?OBJ ?BASE)))
       (instance pddl-action :init
		 :name "pick_f"
		 :parameters '((?OBJ object))
		 :precondition '((forall (?obj - object)
					 (NOT (GRASP ?obj))))
		 :effect '((ON ?OBJ FLOOR)))
       (instance pddl-action :init
		 :name "put_f"
		 :parameters '((?OBJ ?BASE object))
		 :precondition '((GRASP ?OBJ))
		 :effect '((NOT (GRASP ?OBJ))
			   (ON ?OBJ FLOOR)))
       ))

(dolist (ac *actlist*) (send *domain* :add :action ac))

;;
;; problem
;;
(setq *problem*
      (instance pddl-problem :init :name 'sample-task :domain 'pr2_action))

;; three objects
(send *problem* :objects
      '((CUP . object)
        (TABLE . object)
        (FLOOR . object)
      ))

;; initial condition
(send *problem* :initial-condition
      '((ON CUP FLOOR)))

;; we want to cup to be on table
(send *problem* :goal-condition
      '((ON CUP TABLE)))
