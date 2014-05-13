#!/usr/bin/env roseus
;; mostly same to demo-eus.l in pddl package under jsk subversion trunk
(load "package://pddl_planner/src/eus-pddl-client.l")

;;
;; problem
;;
;;hanoi tower
(setq *problem* (instance pddl-problem :init :name 'tecnalia :domain 'manip))

;;使用する変数の宣言
(send *problem* :objects
      '((nat    . object)
        (screw  . object)
	(desk   . object)
	(hole   . object)
	(l-arm  . object)
	(r-arm  . object)
       ))

;;初期条件
(send *problem* :initial-condition
      '((on screw l-arm desk)
        (on nat   r-arm desk)
        ))

;;終了条件
(send *problem* :goal-condition
      '((screwed screw nat) ;;
        ))

;;
;; domain
;;
(setq *domain* (instance pddl-domain :init :name 'manip))
(send *domain* :requirements '(:typing)) ;; ??
(send *domain* :types '(object))
(send *domain* :predicates '((on ?obj ?arm ?from - object)
			     (in ?obj ?arm ?to - object)
                             (clear ?obj - object)
                             (hold ?obj0 ?arm - object)
                             (screwed ?obj0 ?obj1 - object)))
;;making action
(setq *pick* (instance pddl-action :init
                       :name "pick"
                       :parameters '((?obj ?arm ?from object))
                       :precondition '((on ?obj ?arm ?from)
                                       )
                       :effect '((not (on ?obj ?arm ?from))
                                 (clear ?from)
				 (hold ?obj ?arm)
				 )))
(setq *insert* (instance pddl-action :init ;;
                       :name "insert"
                       :parameters '((?obj ?arm ?to object))
                       :precondition '((hold ?obj ?arm)
                                       )
                       :effect '((not (hold ?obj ?arm))
				 (in ?obj ?arm ?to)
				 )))
(setq *screw* (instance pddl-action :init ;;
                       :name "screw"
                       :parameters '((?obj1 ?obj2 object))
                       :precondition '(
				       (in ?obj1 l-arm hole)
				       (in ?obj2 r-arm hole)
				       )
                       :effect '((screwed ?obj1 ?obj2)
				 )))
;;add action to domain
(send *domain* :add :action *pick*)
(send *domain* :add :action *insert*)
(send *domain* :add :action *screw*)

;;
;; solve planning
;;
(setq *result* (solve-pddl-planning *domain* *problem*))
(pprint (cdr (assoc :plan *result*)))
;;(pprint (cdr (assoc :step-state *result*)))
(ros::exit)
