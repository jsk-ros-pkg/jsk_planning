;; solve-simple-failure-torelant.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "simple_metric_plan")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")

;; define domain
(setq *domain* (instance pddl-domain :init :name 'simple_failure_torelant_domain))
(send *domain* :requirements '(:typing))
(send *domain* :types '(button spot))
(send *domain* :predicates
      '((LIGHT ?OBJ - button)
        (FOUND ?OBJ - button)
        (AT ?SPOT - spot)))
(setq *actlist*
      (list
       (instance pddl-action :init
                 :name "move-to"
                 :parameters '((?TO spot))
                 :precondition '((NOT (AT ?TO)))
                 :effect '((forall (?AT - spot)
                            (NOT (AT ?AT)))
                           (AT ?TO)))
       (instance pddl-action :init
                 :name "look"
                 :parameters '((?BTN button))
                 :precondition '((AT ELEVATOR)
                                 (NOT (FOUND ?BTN)))
                 :effect '((FOUND ?BTN)))
       (instance pddl-action :init
                 :name "look_f"
                 :parameters '((?BTN button))
                 :precondition '((AT ELEVATOR)
                                 (NOT (FOUND ?BTN)))
                 :effect '((NOT (AT ELEVATOR))
                           (AT SOMEWHERE)))
       (instance pddl-action :init
                 :name "push-button"
                 :parameters '((?BTN button))
                 :precondition '((FOUND ?BTN))
                 :effect '((LIGHT ?BTN)))
       (instance pddl-action :init
                 :name "push-button_f"
                 :parameters '((?BTN button))
                 :precondition '((FOUND ?BTN))
                 :effect '((NOT (FOUND ?BTN))))
       ))
(dolist (a *actlist*)
  (send *domain* :add :action a))

;; define problem
(setq *problem* (instance pddl-problem :init :name 'push_button :domain 'simple_failure_torelant_domain))
(send *problem* :objects
      '((SOMEWHERE . spot)
        (START . spot)
        (ELEVATOR . spot)
        (ELEVATOR-BUTTON . button)))

(send *problem* :initial-condition
      '((AT START)
        (NOT (FOUND ELEVATOR-BUTTON))
        (NOT (LIGHT ELEVATOR-BUTTON))))

(send *problem* :goal-condition
      '((LIGHT ELEVATOR-BUTTON)))

;; declare defined-failure nodes
(setq *failed-nodes* (list 'push-button 'look))

(when (ros::get-param "~publish_to_smach" nil)
  (require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
  (setq *graph*
        (pddl-plan-to-graph nil :domain *domain* :problem *problem* :failed-nodes *failed-nodes* :debug t))
  (send *graph* :write-to-pdf "/tmp/graph.pdf" nil "simple_failure_torelant")
  (piped-fork "gnome-open /tmp/graph.pdf")
  (require :pddl2smach "package://roseus_smach/src/pddl2smach.l")
  (require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")

  (setq *sm-inspect* (pddl-graph-to-smach *graph*
                                          :add-self-return-failure-path nil))

  (unix::sleep 1)
  (send *sm-inspect* :reset-state)
  )

;; (when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
;;   (ros::exit))

(provide :solve-simple-failure-torelant) ;; end of solve-simple-failure-torelant.l
