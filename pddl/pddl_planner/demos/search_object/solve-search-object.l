#!/usr/bin/env roseus

(load "package://pddl_planner/src/pddl-result-graph.l")
(load "package://pddl_planner/src/eus-pddl-client.l")

;;
;; problem
;;
(setq *problem* (instance pddl-problem :init :name 'searchproblem :domain 'searchobject))

;;使用する変数の宣言
(send *problem* :objects
      '((START . object)
        (END . object)
        (BOXA . object)
        (BOXB . object)
        (BOXC . object)
        ))

;;初期条件
(send *problem* :initial-condition
      '((CLOSED BOXA)
        (CLOSED BOXB)
        (CLOSED BOXC)
        (AT START)
        ))

;;終了条件
(send *problem* :goal-condition
      '((CLOSED BOXA)
        (CLOSED BOXB)
        (CLOSED BOXC)
        (ONHAND)
        (AT END)
        ))
(setq *problem-fail-end* (copy-object *problem*))
(send *problem-fail-end* :goal-condition
      '((CLOSED BOXA)
        (CLOSED BOXB)
        (CLOSED BOXC)
        (NOTEXISTS BOXA)
        (NOTEXISTS BOXB)
        (NOTEXISTS BOXC)
        (AT END)
        ))

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'searchobject))
(send *domain* :requirements '(:typing))
(send *domain* :types '(object spot))
(send *domain* :predicates '((ONHAND)
                             (CLOSED ?BOX - object)
                             (AT ?SPOT - object)
                             (DETECTED ?OBJ - object)
                             (NOTEXISTS ?OBJ - object)
                             ))

;;making action
(setq *action-list*
      (list
       (instance pddl-action :init
                 :name "grasp"
                 :parameters '((?SPOT object))
                 :precondition '((AT ?SPOT)
                                 (DETECTED ?SPOT)
                                 (NOT (CLOSED ?SPOT))
                                 (NOT (ONHAND)))
                 :effect '((ONHAND))
                 )
       (instance pddl-action :init
                 :name "move-to"
                 :parameters '((?SPOT object))
                 :precondition '((NOT (AT ?SPOT))
                                 (CLOSED BOXA)
                                 (CLOSED BOXB)
                                 (CLOSED BOXC))
                 :effect '((forall (?ANY - object) (NOT (AT ?ANY)))
                           (AT ?SPOT))
                 )
       (instance pddl-action :init
                 :name "open-box"
                 :parameters '((?BOX object))
                 :precondition '((AT ?BOX)
                                 (CLOSED ?BOX)
                                 (NOT (ONHAND)))
                 :effect '((NOT (CLOSED ?BOX)))
                 )
       (instance pddl-action :init
                 :name "close-box"
                 :parameters '((?BOX object))
                 :precondition '((AT ?BOX)
                                 (NOT (CLOSED ?BOX)))
                 :effect '((CLOSED ?BOX))
                 )
       (instance pddl-action :init
                 :name "detect"
                 :parameters '((?BOX object))
                 :precondition '((AT ?BOX)
                                 (NOT (= ?BOX START))
                                 (NOT (= ?BOX END))
                                 (NOT (NOTEXISTS ?BOX))
                                 (NOT (CLOSED ?BOX))
                                 (NOT (ONHAND)))
                 :effect '((DETECTED ?BOX))
                 )
       (instance pddl-action :init
                 :name "detect_f"
                 :parameters '((?BOX object))
                 :precondition '((AT ?BOX)
                                 (NOT (= ?BOX START))
                                 (NOT (= ?BOX END))
                                 (NOT (NOTEXISTS ?BOX))
                                 (NOT (CLOSED ?BOX))
                                 (NOT (ONHAND)))
                 :effect '((NOTEXISTS ?BOX))
                 )
       ))
;;add action to domain
(dolist (act *action-list*)
  (send *domain* :add :action act))

;;
;; solve planning
;;
(pprint (setq *result* (solve-pddl-planning *domain* *problem* :debug nil)))
(pprint (setq *result-fail-end* (solve-pddl-planning *domain* *problem-fail-end* :debug nil)))

(setq *graph* (make-graph-from-pddl-results (list *result-fail-end*) :node-name :pprint))
(setq *graph* (add-failed-nodes *result* (list 'detect) :graph *graph*))
#| original
(pprint (setq *result* (solve-pddl-planning *domain* *problem* :debug nil)))
(setq *graph* (add-failed-nodes *result* (list 'detect)))
|#
(make-readable-graph *graph* :copy nil)

(send *graph* :write-to-pdf "debug.pdf")

(when (and (ros::has-param "~display_graph")
           (ros::get-param "~display_graph"))
  (piped-fork "gnome-open debug.pdf")
  (load "package://roseus_smach/src/pddl2smach.l")
  (setq sm (convert-smach *graph* :return-success :success :return-fail :fail
                          :add-self-return-failure-path nil))

  (load "package://roseus_smach/src/state-machine-ros.l")
  (setq sm-inspect (instance state-machine-inspector :init sm))
  (unix::sleep 1)
  (send sm :reset-state)
  (progn
    (send sm-inspect :publish-structure)
    (send sm-inspect :publish-status nil))
  )

(when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
  (ros::exit))
