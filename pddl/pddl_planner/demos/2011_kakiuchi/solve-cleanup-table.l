#!/usr/bin/env roseus

(load "package://pddl_planner/src/pddl-result-graph.l")
(load "package://pddl_planner/src/eus-pddl-client.l")

;;
;; problem
;;
(setq *problem* (instance pddl-problem :init :name 'cleanup_table :domain 'kakiuchi_robomec))

;;使用する変数の宣言
(send *problem* :objects
      '((START . spot)
        (TABLE . spot)
        (TRASHBOX . spot)
        (TRASH . object)
        ;;(TRASH1 . object)
        ;;(TRASH2 . object)
        ;;(NULL . object)
        ))

;;初期条件
(send *problem* :initial-condition
      '((NOT (SEARCHED))
        (NOT (ONHAND TRASH))
        (AT START)
        ;;(ON TRASH TABLE)
        ))

;;終了条件
(send *problem* :goal-condition
      '(
        ;;(ON TRASH TRASHBOX) ;; trash on trash
        ;;(AT START)
        (CLEARCHECKED)
        ))

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'kakiuchi_robomec))
(send *domain* :requirements '(:typing))
(send *domain* :types '(object spot))
(send *domain* :predicates '((ON ?OBJ - object ?SPOT - spot)
                              (ONHAND ?OBJ - object)
                              (AT ?SPOT - spot)
                              (KNOWNPOS ?SPOT - spot)
                              (POSUPDATED ?SPOT - spot)
                              (CLEARCHECKED)
                              (SEARCHED)
                              ))

;;making action
(setq *action-list*
      (list
       (instance pddl-action :init
                 :name "pick"
                 :parameters '((?OBJ object) (?SPOT spot))
                 :precondition '((SEARCHED)
                                 (ON ?OBJ ?SPOT)
                                 (NOT (ONHAND ?OBJ))
                                 (AT ?SPOT))
                 :effect '((NOT (ON ?OBJ ?SPOT))
                           (ONHAND ?OBJ)
                           (forall (?S - spot)
                                   (when (AND (POSUPDATED ?S))
                                     (AND (NOT (POSUPDATED ?S)))))))
#|
       (instance pddl-action :init
                 :name "put"
                 :parameters '((?OBJ object) (?SPOT spot))
                 :precondition '((SEARCHED)
                                 (ONHAND ?OBJ)
                                 (AT ?SPOT))
                 :effect '((ON ?OBJ ?SPOT)
                           (NOT (ONHAND ?OBJ))
                           (forall (?S - spot)
                                   (when (AND (POSUPDATED ?S))
                                     (AND (NOT (POSUPDATED ?S)))))))
|#
       (instance pddl-action :init
                 :name "dispose"
                 :parameters '()
                 :precondition '((SEARCHED)
                                 (ONHAND TRASH)
                                 (AT TRASHBOX))
                 :effect '((ON TRASH TRASHBOX)
                           (NOT (ONHAND TRASH))
                           (forall (?S - spot)
                                   (when (AND (POSUPDATED ?S))
                                     (AND (NOT (POSUPDATED ?S)))))))
       (instance pddl-action :init
                 :name "move"
                 :parameters '((?FROM ?TO spot))
                 :precondition '((SEARCHED)
                                 (AT ?FROM)
                                 (KNOWNPOS ?TO)
                                 (POSUPDATED ?TO)
                                 (NOT (= ?FROM ?TO)))
                 :effect '((AT ?TO)
                           (NOT (AT ?FROM))
                           (forall (?S - spot)
                                   (when (AND (POSUPDATED ?S))
                                     (AND (NOT (POSUPDATED ?S)))))))

       (instance pddl-action :init
                 :name "checkclear"
                 :parameters '()
                 :precondition '((SEARCHED)
                                 (AT TABLE)
                                 (NOT (ONHAND TRASH))
                                 (NOT (ON TRASH TABLE)))
                 ;;(ON TRASH TRASHBOX))
                 :effect '((CLEARCHECKED)
                           (forall (?S - spot)
                                   (when (AND (POSUPDATED ?S))
                                     (AND (NOT (POSUPDATED ?S)))))))

       (instance pddl-action :init
                 :name "checkclear_f"
                 :parameters '()
                 :precondition '((SEARCHED)
                                 (AT TABLE)
                                 (NOT (ONHAND TRASH))
                                 (NOT (ON TRASH TABLE)))
                 :effect '(;;(CLEARCHECKED)
                           (ON TRASH TABLE)
                           (NOT (ON TRASH TRASHBOX))
                           (forall (?S - spot)
                                   (when (AND (POSUPDATED ?S))
                                     (AND (NOT (POSUPDATED ?S)))))))

       (instance pddl-action :init
                 :name "confirm"
                 :parameters '((?POS spot))
                 :precondition '((SEARCHED)
                                 (KNOWNPOS ?POS)
                                 (NOT (POSUPDATED ?POS)))
                 :effect '((POSUPDATED ?POS)))

       (instance pddl-action :init
                 :name "confirm_f"
                 :parameters '((?POS spot))
                 :precondition '((SEARCHED)
                                 (KNOWNPOS ?POS)
                                 (NOT (POSUPDATED ?POS)))
                 :effect '((NOT (SEARCHED))
                           (forall (?S - spot)
                                   (AND (NOT (KNOWNPOS ?S))))))

       (instance pddl-action :init
                 :name "search"
                 :parameters '()
                 :precondition '((NOT (SEARCHED)))
                 :effect '((SEARCHED)
                           (forall (?S - spot)
                                   (AND (KNOWNPOS ?S)
                                        (POSUPDATED ?S)))))
       ))
;;add action to domain
(dolist (act *action-list*)
  (send *domain* :add :action act))

;;
;; solve planning
;;
(pprint (setq *result* (solve-pddl-planning *domain* *problem* :debug nil)))

(setq gr (make-graph-from-pddl-results (list *result*) :node-name :pprint))

(setq *graph* (add-failed-nodes *result* (list 'checkclear 'confirm)))
;; swap nodes ( confirm <-> confirm_f and checkclear <-> checkclear_f ) for making task flow

(send *graph* :write-to-pdf "kakiuchi_robomec.pdf")

(when (ros::get-param "~display_graph")
  (piped-fork "gnome-open kakiuchi_robomec.pdf"))

(when (string= "__log:=" (subseq (car (last lisp::*eustop-argument*)) 0 7))
  (ros::exit))
